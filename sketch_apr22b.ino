/*
 * Component Tester для ATMEGA8
 * Подключение по вашей схеме:
 * LCD: D4=PD0, D5=PD1, D6=PD2, D7=PD3, RS=PD4, E=PD5, RW=GND
 * Управление: PD6 (LED), PD7 (измерение с Q4)
 * Измерение: PC0, PC1, PC2 через резисторы PB0-PB5
 */

// Подключаем AVR заголовки
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

// ========== LCD ПИНЫ ==========
#define LCD_D4_PIN    PD0
#define LCD_D5_PIN    PD1
#define LCD_D6_PIN    PD2
#define LCD_D7_PIN    PD3
#define LCD_RS_PIN    PD4
#define LCD_E_PIN     PD5

// ========== УПРАВЛЕНИЕ ==========
#define LED_CTRL_PIN  PD6   // Управление Q3 (LED)
#define MEASURE_PIN   PD7   // Чтение с Q4

// ========== ИЗМЕРИТЕЛЬНЫЕ РЕЗИСТОРЫ ==========
#define R6_PIN        PB1   // 470k
#define R7_PIN        PB0   // 680
#define R8_PIN        PB3   // 470k
#define R9_PIN        PB2   // 680
#define R10_PIN       PB5   // 470k
#define R15_PIN       PB4   // 680

// ========== ИЗМЕРИТЕЛЬНЫЕ ВХОДЫ ==========
#define MEASURE0_PIN  PC0
#define MEASURE1_PIN  PC1
#define MEASURE2_PIN  PC2

// ========== ФУНКЦИИ LCD ==========
void lcd_nibble(uint8_t nibble, uint8_t is_data) {
    if (is_data) PORTD |= (1 << LCD_RS_PIN);
    else PORTD &= ~(1 << LCD_RS_PIN);
    
    if (nibble & 0x01) PORTD |= (1 << LCD_D4_PIN); else PORTD &= ~(1 << LCD_D4_PIN);
    if (nibble & 0x02) PORTD |= (1 << LCD_D5_PIN); else PORTD &= ~(1 << LCD_D5_PIN);
    if (nibble & 0x04) PORTD |= (1 << LCD_D6_PIN); else PORTD &= ~(1 << LCD_D6_PIN);
    if (nibble & 0x08) PORTD |= (1 << LCD_D7_PIN); else PORTD &= ~(1 << LCD_D7_PIN);
    
    PORTD |= (1 << LCD_E_PIN);
    _delay_us(2);
    PORTD &= ~(1 << LCD_E_PIN);
    _delay_us(50);
}

void lcd_cmd(uint8_t cmd) {
    lcd_nibble(cmd >> 4, 0);
    lcd_nibble(cmd & 0x0F, 0);
    if (cmd == 0x01 || cmd == 0x02) _delay_ms(2);
    else _delay_us(100);
}

void lcd_data(uint8_t data) {
    lcd_nibble(data >> 4, 1);
    lcd_nibble(data & 0x0F, 1);
    _delay_us(50);
}

void lcd_init(void) {
    _delay_ms(50);
    
    DDRD |= (1 << LCD_D4_PIN) | (1 << LCD_D5_PIN) | (1 << LCD_D6_PIN) | (1 << LCD_D7_PIN);
    DDRD |= (1 << LCD_RS_PIN) | (1 << LCD_E_PIN);
    
    _delay_ms(15);
    lcd_nibble(0x03, 0); _delay_ms(5);
    lcd_nibble(0x03, 0); _delay_ms(1);
    lcd_nibble(0x03, 0); _delay_ms(1);
    lcd_nibble(0x02, 0); _delay_ms(1);
    
    lcd_cmd(0x28);  // 4-bit, 2 строки
    lcd_cmd(0x0C);  // Дисплей вкл, курсор выкл
    lcd_cmd(0x01);  // Очистка
    _delay_ms(2);
    lcd_cmd(0x06);  // Сдвиг вправо
}

void lcd_string(const char *str) {
    while (*str) lcd_data(*str++);
}

void lcd_gotoxy(uint8_t x, uint8_t y) {
    uint8_t addr = (y == 0) ? 0x80 + x : 0xC0 + x;
    lcd_cmd(addr);
}

void lcd_clear(void) {
    lcd_cmd(0x01);
    _delay_ms(2);
}

// ========== ADC ==========
void init_adc(void) {
    ADMUX = (1 << REFS0);  // AVCC как опорное
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // Включить, делитель 64
}

uint16_t read_adc(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);  // Запуск преобразования
    while (ADCSRA & (1 << ADSC));  // Ждем завершения
    return ADC;
}

// ========== УПРАВЛЕНИЕ РЕЗИСТОРАМИ ==========
void set_resistors(uint8_t r6, uint8_t r7, uint8_t r8, uint8_t r9, uint8_t r10, uint8_t r15) {
    if (r6) PORTB |= (1 << R6_PIN); else PORTB &= ~(1 << R6_PIN);
    if (r7) PORTB |= (1 << R7_PIN); else PORTB &= ~(1 << R7_PIN);
    if (r8) PORTB |= (1 << R8_PIN); else PORTB &= ~(1 << R8_PIN);
    if (r9) PORTB |= (1 << R9_PIN); else PORTB &= ~(1 << R9_PIN);
    if (r10) PORTB |= (1 << R10_PIN); else PORTB &= ~(1 << R10_PIN);
    if (r15) PORTB |= (1 << R15_PIN); else PORTB &= ~(1 << R15_PIN);
}

void all_resistors_off(void) {
    set_resistors(0,0,0,0,0,0);
}

// ========== ИНИЦИАЛИЗАЦИЯ ==========
void init_tester(void) {
    // Резисторы как выходы
    DDRB |= (1 << R6_PIN) | (1 << R7_PIN) | (1 << R8_PIN) | (1 << R9_PIN) | (1 << R10_PIN) | (1 << R15_PIN);
    all_resistors_off();
    
    // Измерительные входы (PC0-PC2)
    DDRC &= ~((1 << MEASURE0_PIN) | (1 << MEASURE1_PIN) | (1 << MEASURE2_PIN));
    PORTC &= ~((1 << MEASURE0_PIN) | (1 << MEASURE1_PIN) | (1 << MEASURE2_PIN));
    
    // Управление LED (PD6) как выход
    DDRD |= (1 << LED_CTRL_PIN);
    PORTD &= ~(1 << LED_CTRL_PIN);
    
    // PD7 как вход для измерения с Q4
    DDRD &= ~(1 << MEASURE_PIN);
    PORTD &= ~(1 << MEASURE_PIN);
}

// ========== ИЗМЕРЕНИЕ ==========
uint32_t measure_resistance(uint8_t channel, uint32_t r_ref, uint8_t r_pin) {
    all_resistors_off();
    _delay_ms(10);
    
    // Включаем опорный резистор
    PORTB |= (1 << r_pin);
    _delay_ms(10);
    
    uint16_t adc_val = read_adc(channel);
    
    PORTB &= ~(1 << r_pin);
    
    if (adc_val == 0) return 999999;  // Обрыв
    if (adc_val >= 1023) return 0;    // КЗ
    
    // R = (1023 / ADC - 1) * R_ref
    uint32_t resistance = ((1023UL * r_ref) / adc_val) - r_ref;
    return resistance;
}

// ========== ГЛАВНАЯ ФУНКЦИЯ ==========
int main(void) {
    // Для Arduino IDE настройка тактирования не нужна, 
    // она задается в настройках платы (8 MHz internal)
    
    init_adc();
    lcd_init();
    init_tester();
    
    char buffer[17];
    
    // Приветствие
    lcd_clear();
    lcd_gotoxy(0, 0);
    lcd_string("Component Tester");
    lcd_gotoxy(0, 1);
    lcd_string("Ready");
    
    // Мигаем LED 2 раза
    for (uint8_t i = 0; i < 2; i++) {
        PORTD |= (1 << LED_CTRL_PIN);
        _delay_ms(200);
        PORTD &= ~(1 << LED_CTRL_PIN);
        _delay_ms(200);
    }
    
    _delay_ms(1000);
    
    while (1) {
        lcd_clear();
        lcd_gotoxy(0, 0);
        lcd_string("Measuring PC0...");
        
        // Измерение на PC0 через R6 (470k)
        uint32_t r1 = measure_resistance(0, 470000, R6_PIN);
        
        lcd_clear();
        lcd_gotoxy(0, 0);
        lcd_string("R=");
        
        if (r1 >= 1000000) {
            ultoa(r1 / 1000000, buffer, 10);
            lcd_string(buffer);
            lcd_string("M");
        } else if (r1 >= 1000) {
            ultoa(r1 / 1000, buffer, 10);
            lcd_string(buffer);
            lcd_string("k");
        } else {
            ultoa(r1, buffer, 10);
            lcd_string(buffer);
        }
        lcd_string(" Ohm");
        
        lcd_gotoxy(0, 1);
        uint16_t adc = read_adc(0);
        itoa(adc, buffer, 10);
        lcd_string("ADC=");
        lcd_string(buffer);
        
        _delay_ms(3000);
    }
}
