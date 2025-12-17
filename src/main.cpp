#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c.h"
#include "mpu6050.h"
#include "uart.h"
#include "rc522.h"
#include "st7920.h"
//#include "st7920_graphics.h"
#include "mmc_avr.h"
#include "ff.h"

// Definicao de pinos do encoder, todos são proximos fisicamente, mas ficaram em ports diferentes
#define ENCODER_PIN_A   PG2
#define ENCODER_PIN_B   PC0
#define ENCODER_BUTTON  PD7
#define ENCODER_A_DDR   DDRG
#define ENCODER_A_PORT  PORTG
#define ENCODER_A_PIN   PING
#define ENCODER_B_DDR   DDRC
#define ENCODER_B_PORT  PORTC
#define ENCODER_B_PIN   PINC
#define ENCODER_BUTTON_DDR  DDRD
#define ENCODER_BUTTON_PORT PORTD
#define ENCODER_BUTTON_PIN  PIND

#define OSC_A_PIN    PH4
#define OSC_A_DDR   DDRH
#define OSC_A_PORT  PORTH
#define OSC_B_PIN    PH6
#define OSC_B_DDR   DDRH
#define OSC_B_PORT  PORTH

#define TOGGLE_A() OSC_A_PORT ^= (1 << OSC_A_PIN)
#define SET_A()    OSC_A_PORT |= (1 << OSC_A_PIN)
#define CLEAR_A()  OSC_A_PORT &= ~(1 << OSC_A_PIN)
#define TOGGLE_B() OSC_B_PORT ^= (1 << OSC_B_PIN)
#define SET_B()    OSC_B_PORT |= (1 << OSC_B_PIN)
#define CLEAR_B()  OSC_B_PORT &= ~(1 << OSC_B_PIN)

// Estados da maquina de estados principal
typedef enum {
    FSM_START,
    FSM_LOAD_SYSTEM,
    FSM_MENU,
    FSM_WAIT_PLAYER,
    FSM_WAIT_MOVE,
    FSM_SHOW_RESULT
} fsmState_t;

// Estruturas de dados relacionadas ao menu e jogadores que eh lida do SD e movida
typedef struct {
    uint8_t id;
    char name[32];
    char formula[32];
    uint8_t type; // 0=normal, 1=player_mod, 2=check
} MenuOption;

typedef struct {
    char uid[16];
    char name[16];
    uint8_t attr[6], ac;
} PlayerData;

// Funcoes auxiliares feitas de maneira reduzida
static inline int isdigit(char c) {
    return (c >= '0' && c <= '9');
}
static inline int isxdigit(char c) {
    return ((c >= '0' && c <= '9') ||
            (c >= 'a' && c <= 'f') ||
            (c >= 'A' && c <= 'F'));
}
int atoi(const char *s)
{
    int sign = 1;
    int value = 0;

    if (*s == '-') {
        sign = -1;
        s++;
    }

    while (isdigit(*s)) {
        value = value * 10 + (*s - '0');
        s++;
    }

    return value * sign;
}


// Funcoes principais do sistema
void mpu6050_callback(uint8_t result);

void mpu6050_initProcess(void);

int load_config_file(const char *filename);

int parse_formula(const char *str, PlayerData *p);

int get_player_stat(PlayerData *p, const char *stat);

int roll(int n, int d) {
    int total = 0;
    for(int i = 0; i < n; i++) {
        total += (rand() % d) + 1;
    }
    return total;
}

// Variaveis globais, uma por conta de interrupcao, e as outras por serem chamadas em funcoes
volatile uint8_t motionDetected = 0;

MenuOption options[16];
PlayerData players[16];
uint8_t num_options = 0;
uint8_t num_players = 0;

int main(void)
{
    static FATFS sdCard;
    FRESULT result;
    static FIL file;
    UINT bytesProcessed = 0;
    static char auxBuf[64];
    fsmState_t fsmState = FSM_START;
    uint8_t once = 0, moving = 0, tst = 0;
    PlayerData *currentPlayer = NULL;

    uint8_t state = 0, currentA, currentB, lastState = 0;

    uint8_t tagType[2];
    uint8_t uid[5];
    char fileBuf[13];

    cli();

    // Configuracao do encoder
    ENCODER_A_DDR &= ~(1 << ENCODER_PIN_A);
    ENCODER_A_PORT |= (1 << ENCODER_PIN_A);
    ENCODER_B_DDR &= ~(1 << ENCODER_PIN_B);
    ENCODER_B_PORT |= (1 << ENCODER_PIN_B);
    ENCODER_BUTTON_DDR &= ~(1 << ENCODER_BUTTON);
    ENCODER_BUTTON_PORT |= (1 << ENCODER_BUTTON);

    OSC_A_DDR |= (1 << OSC_A_PIN);
    OSC_B_DDR |= (1 << OSC_B_PIN);

    // Ajuste dos pinos de dispositivos externos como a SPI bit-bang e o RFID
    RC522_CS_DDR |= (1 << RC522_CS_PIN);
    rc522_deactivate();
    DDRE |= (1 << PE5);
    DDRL |= (1 << PL7) | (1 << PL6);
    DDRA |= (1 << PA7) | (1 << PA1);
    PORTA |= (1 << PA7); 

    // Timer 0 para uso no seeding do RNG
    TCCR0A = 0;
    TCCR0B = (1 << CS00);

    // Timer 1 para interrupcao necessaria para o cartao SD
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12); 
    OCR1A = 624;
    TIMSK1 = (1 << OCIE1A);

    // UART eh usada para debug, I2C para o MPU6050, SPI para o RC522 e o SD e bit-bang para o ST7920
    uart_init(9600);
    i2c_init(250000); // 250kHz I2C speed
    uart_send_string("System initializing...\r\n");
    mpu6050_init();
    spi_init(SPI_CLOCK_DIV64, SPI_MODE0, SPI_MSB_FIRST);
    rc522_init();
    rc522_deactivate();
    st7920_init();

    // Interrupcao externa para detectar movimento via MPU6050
    EICRB = (1 << ISC41);
    EIMSK = (1 << INT4);
    DDRE &= ~(1 << PE4);

    i2c_setCallback(mpu6050_callback);

    sei();

    // Montagem do cartao SD com arquivo de teste de log, mantido so para demonstrar um teste de escrita
    uart_send_string("Mounting SD card...\r\n");
    result = f_mount(&sdCard, "", 1);
    if(result != FR_OK) {
        uart_send_string("Failed to mount SD card\r\n");
        sprintf(auxBuf, "f_mount error: %d\r\n", result);
        uart_send_string(auxBuf);
    } else {
        uart_send_string("SD card mounted successfully\r\n");

        result = f_open(&file, "log.txt", FA_WRITE | FA_OPEN_ALWAYS);
        if(result != FR_OK) {
            uart_send_string("Failed to open log.txt\r\n");
        } else {
            uart_send_string("log.txt opened successfully\r\n");

            f_lseek(&file, f_size(&file));

            const char *initMsg = "Log start\r\n";
            f_write(&file, initMsg, strlen(initMsg), &bytesProcessed);

            f_close(&file);
        }
    }

    uart_send_string("MPU6050 test starting...\r\n");

    mpu6050_initProcess();
    
    uart_send_string("Initialization complete.\r\n");

    //st7920_print("ABCD");

    _delay_ms(1000);

    rc522_activate();

    st7920_cmd(DISP_CLEAR);
    st7920_cmd(DISP_HOME);

    // Parte grafica desativada para MVP
    /*gfx_init();
    gfx_draw_d10(30, 0, 60, 1, 1);
    gfx_update();*/
    st7920_disableCursor();
    

    uint8_t menuTop = 0;
    uint8_t menuSelected = 0;

    // --- Loop reading accel + gyro ---
    while(1)
    {
        // Caso o estado nao exiga movimento, limpa o flag
        if(motionDetected && fsmState != FSM_WAIT_MOVE) {
            motionDetected = 0;
            mpu6050_readInterruptStatus();
            uart_send_string("Motion detected!\r\n");
        }
        
        // Maquina de estados principal
        switch (fsmState)
        {
        case FSM_START:
            if(!tst) {
                SET_A();
            }
            if(!once) {
                st7920_cmd(DISP_CLEAR);
                st7920_print(" SCAN SYSTEM CARD");
                st7920_disableCursor();
                once = 1;
            }
            
            // Leitura do RFID, feita de uma forma padrao 
            if (rc522_request(PICC_REQIDL, tagType) == 0)
            {
                uart_send_string("Card detected!\r\n");

                rc522_writeReg(RC522_REG_COMMAND, RC522_CMD_IDLE);
                rc522_writeReg(RC522_REG_COMM_IRQ, 0x7F);            
                rc522_writeReg(RC522_REG_FIFO_LEVEL, 0x80);

                if (rc522_anticoll(uid) == 0)
                {
                    sprintf(auxBuf, "UID: %02X %02X %02X %02X %02X\r\n",
                            uid[0], uid[1], uid[2], uid[3], uid[4]);
                    uart_send_string(auxBuf);

                    sprintf(fileBuf, "%02X%02X%02X%02X.txt",
                            uid[0], uid[1], uid[2], uid[3]);
                    tst = 0;
                    fsmState = FSM_LOAD_SYSTEM;
                }
                rc522_writeReg(RC522_REG_COMM_IRQ, 0x7F);  
                _delay_ms(300);                                      // Debounce card
            }
            if(!tst) {
                CLEAR_A();
                if(fsmState == FSM_START)
                    tst = 1;
            }
            break;
        case FSM_LOAD_SYSTEM:
            if(!tst) {
                SET_A();
            }
            // Carrega o sistema do cartao SD
            result = f_open(&file, fileBuf, FA_READ);
            if(result != FR_OK) {
                uart_send_string("Failed to open system file\r\n");
                st7920_cmd(DISP_CLEAR);
                st7920_print(" LOAD FAILED");
                st7920_disableCursor();
                _delay_ms(1000);
                tst = 0;
                fsmState = FSM_START;
                once = 0;
            } else {
                uart_send_string("System file opened successfully\r\n");
                
                f_close(&file);

                load_config_file(fileBuf);
                uart_send_string("System configuration loaded\r\n");
                st7920_cmd(DISP_CLEAR);
                st7920_print(" SYSTEM LOADED");
                st7920_disableCursor();
                _delay_ms(1000);
                // Gera a seed utilizada para o RNG a partir do Timer0
                srand(TCNT0); 
                TCCR0B = 0; 
                once = 0;
                tst = 0;
                fsmState = FSM_MENU;
            }
            if(!tst) {
                CLEAR_A();
                if(fsmState == FSM_LOAD_SYSTEM)
                    tst = 1;
            }
            break;
            // Mostra o menu principal e lida com o encoder rotativo
        case FSM_MENU:
            if(!tst) {
                SET_A();
            }
            if(!once) {
                st7920_cmd(DISP_CLEAR);
                st7920_cmd(DISP_HOME);
                // Imprime duas opcoes do menu por vez, por limitacao do display
                for(int8_t i = menuTop; i < menuTop + 2 && i < num_options; i++) {
                    if(i == menuSelected) {
                        st7920_print(" >");
                    } else {
                        st7920_print("  ");
                    }
                    st7920_print(options[i].name);
                    st7920_cmd(DISP_DDRAM_ADDR(0x40 + (i+1 - menuTop) * 16));
                    uart_send_string("Menu option displayed\r\n");
                    uart_send_string(options[i].name);
                    uart_send_string("\r\n");
                }
                st7920_disableCursor();
                once = 1;
            }
            if (!(ENCODER_BUTTON_PIN & (1 << ENCODER_BUTTON))) {
                uart_send_string("Menu option selected: ");
                uart_send_string(options[menuSelected].name);
                uart_send_string("\r\n");

                st7920_cmd(DISP_CLEAR);
                st7920_print(" OPTION SELECTED");
                st7920_disableCursor();
                _delay_ms(300);

                once = 0;
                tst = 0;
                if(options[menuSelected].type != 0) {
                    fsmState = FSM_WAIT_PLAYER;
                } else {
                    fsmState = FSM_WAIT_MOVE;
                }
            }

            // Leitura do encoder rotativo
            lastState = 0;
            currentA = (ENCODER_A_PIN & (1 << ENCODER_PIN_A)) != 0;
            currentB = (ENCODER_B_PIN & (1 << ENCODER_PIN_B)) != 0;

            // Gera um estado de 2 bits (A como bit0, B como bit1)
            state = (currentA << 1) | currentB;

            if (state != lastState) {
                _delay_ms(4);

                currentA = (ENCODER_A_PIN & (1 << ENCODER_PIN_A)) != 0;
                currentB = (ENCODER_B_PIN & (1 << ENCODER_PIN_B)) != 0;
                uint8_t stable = (currentA << 1) | currentB;

                if (stable != lastState) {

                    if (lastState == 0b00) {
                        if (stable == 0b01) {
                            if (menuSelected < num_options - 1) {
                                menuSelected++;
                                if (menuSelected >= menuTop + 2)
                                    menuTop++;

                                uart_send_string("Menu moved down\r\n");
                                once = 0;
                            }
                        } else if (stable == 0b10) { 
                            if (menuSelected > 0) {
                                menuSelected--;
                                if (menuSelected < menuTop)
                                    menuTop--;

                                uart_send_string("Menu moved up\r\n");
                                once = 0;
                            }
                        }
                    }

                    lastState = stable;
                }
            }
            if(!tst) {
                CLEAR_A();
                if(fsmState == FSM_MENU)
                    tst = 1;
            }
            break;
        // Mais uma leitura de RFID
        case FSM_WAIT_PLAYER:
            if(!tst) {
                SET_A();
            }
            if(!once) {
                st7920_cmd(DISP_CLEAR);
                st7920_print(" SCAN PLAYER CARD");
                st7920_disableCursor();
                once = 1;
            }
            if (rc522_request(PICC_REQIDL, tagType) == 0)
            {
                uart_send_string("Player card detected!\r\n");

                rc522_writeReg(RC522_REG_COMMAND, RC522_CMD_IDLE);   // Stop
                rc522_writeReg(RC522_REG_COMM_IRQ, 0x7F);            // Clear interrupts
                rc522_writeReg(RC522_REG_FIFO_LEVEL, 0x80);

                if (rc522_anticoll(uid) == 0)
                {
                    sprintf(auxBuf, "Player UID: %02X %02X %02X %02X %02X\r\n",
                            uid[0], uid[1], uid[2], uid[3], uid[4]);
                    uart_send_string(auxBuf);

                    for(int i = 0; i < num_players; i++) {
                        char playerUID[11];
                        sprintf(playerUID, "%02X%02X%02X%02X%02X",
                                uid[0], uid[1], uid[2], uid[3], uid[4]);
                        if(strcmp(players[i].uid, playerUID) == 0) {
                            currentPlayer = &players[i];
                            break;
                        }
                    }
                    if(currentPlayer == NULL) {
                        uart_send_string("Player not found!\r\n");
                        st7920_cmd(DISP_CLEAR);
                        st7920_print(" PLAYER NOT FOUND");
                        st7920_disableCursor();
                        _delay_ms(1000);
                        once = 0;
                    } else {
                        uart_send_string("Player loaded: ");
                        uart_send_string(currentPlayer->name);
                        uart_send_string("\r\n");
                        st7920_cmd(DISP_CLEAR);
                        st7920_print(" PLAYER LOADED");
                        st7920_disableCursor();
                        _delay_ms(1000);
                        tst = 0;
                        fsmState = FSM_WAIT_MOVE;
                        once = 0;
                    }
                    
                }
                rc522_writeReg(RC522_REG_COMM_IRQ, 0x7F);  
                _delay_ms(300);                                      // Debounce card
            }
            if(!tst) {
                CLEAR_A();
                if(fsmState == FSM_WAIT_PLAYER)
                    tst = 1;
            }
            break;
        case FSM_WAIT_MOVE:
            if(!tst) {
                SET_A();
            }
            if(!once) {
                st7920_cmd(DISP_CLEAR);
                st7920_print(" SHAKE DEVICE");
                st7920_disableCursor();
                once = 1;
            }
            if(motionDetected) {
                CLEAR_B();
                motionDetected = 0;
                mpu6050_readInterruptStatus();
                uart_send_string("Motion detected in WAIT_MOVE!\r\n");
                st7920_cmd(DISP_CLEAR);
                st7920_print(" MOTION DETECTED");
                st7920_disableCursor();
                _delay_ms(1000);
                once = 0;
                tst = 0;
                fsmState = FSM_SHOW_RESULT;
            }
            if(!tst) {
                CLEAR_A();
                if(fsmState == FSM_WAIT_MOVE)
                    tst = 1;
            }
            break;
        case FSM_SHOW_RESULT:
            if(!tst) {
                SET_A();
            }
            if(!once) {
                if(options[menuSelected].type == 0) {
                    st7920_cmd(DISP_CLEAR);
                    st7920_print(" ROLLING...");
                    st7920_disableCursor();
                    _delay_ms(1000);
                }
                st7920_cmd(DISP_CLEAR);
                st7920_print(" ROLL RESULT:");
                int result = parse_formula(options[menuSelected].formula, currentPlayer);
                char resultStr[16];
                sprintf(resultStr, " %d", result);
                st7920_cmd(DISP_DDRAM_ADDR(0x40+16));
                if(options[menuSelected].type == 2) {
                    if(result >= currentPlayer->ac) {
                        st7920_print(" SUCCESS");
                    } else {
                        st7920_print(" FAILURE");
                    }
                    once = 1;
                } else {
                    st7920_print(resultStr);
                    st7920_disableCursor();
                    uart_send_string(resultStr);
                    uart_send_string("\r\n");
                    once = 1;
                }
            }
            if(!(ENCODER_BUTTON_PIN & (1 << ENCODER_BUTTON))) {
                st7920_cmd(DISP_CLEAR);
                st7920_print(" RETURNING...");
                st7920_disableCursor();
                _delay_ms(500);
                tst = 0;
                once = 0;
                currentPlayer = NULL;
                fsmState = FSM_MENU;
            }
            if(!tst) {
                CLEAR_A();
                if(fsmState == FSM_SHOW_RESULT)
                    tst = 1;
            }
            break;
        default:
            break;
        }
    }
}

// Processo longo e completo, usa o assync so por demonstracao
void mpu6050_initProcess(void) {
    // --- Read WHO_AM_I ---
    uint8_t whoami = 0;
    if (mpu6050_whoAmI(&whoami))
    {
        while(mpu6050_isBusy()); // wait for completion
    }
    else
        uart_send_string("WHO_AM_I read failed!\r\n");

    if(mpu6050_setAccelRange(MPU6050_ACCEL_RANGE_2G)) {
        while(mpu6050_isBusy());
        uart_send_string("Set Accel Range to 2G\r\n");
    }
    else
        uart_send_string("Failed to set Accel Range\r\n");

    if(mpu6050_setGyroRange(MPU6050_GYRO_RANGE_250DPS)) {
        while(mpu6050_isBusy());
        uart_send_string("Set Gyro Range to 250DPS\r\n");
    } 
    else
        uart_send_string("Failed to set Gyro Range\r\n");

    if(mpu6050_setMotionDetectionThreshold((uint8_t)10)) {
        while(mpu6050_isBusy());
        uart_send_string("Set Motion Detection Threshold to 10\r\n");
    } 
    else
        uart_send_string("Failed to set Motion Detection Threshold\r\n");
    
    if(mpu6050_setMotionDetectionDuration((uint8_t)1)) {
        while(mpu6050_isBusy());
        uart_send_string("Set Motion Detection Duration to 40\r\n");
    } 
    else
        uart_send_string("Failed to set Motion Detection Duration\r\n");
    if(mpu6050_enableMotionInterrupt()) {
        while(mpu6050_isBusy());
        uart_send_string("Enabled Motion Interrupt\r\n");
    } 
    else
        uart_send_string("Failed to enable Motion Interrupt\r\n");

    if(mpu6050_setMotionDetectionControl(MPU6050_COUNTER_DECREMENT_RESET)) {
        while(mpu6050_isBusy());
        uart_send_string("Modified Motion Counter\r\n");
    } 
    else
        uart_send_string("Failed to modify Motion Counter\r\n");
    if(mpu6050_setAccelOnDelay((uint8_t)1)) {
        while(mpu6050_isBusy());
        uart_send_string("Set Accel On Delay\r\n");
    } 
    else
        uart_send_string("Failed to set Accel On Delay\r\n");

    mpu6050_readInterruptStatus();
}

// Callback que foi definido para o I2C, mas nao eh usado nesse exemplo
void mpu6050_callback(uint8_t result) {
   
}

// Carrega do arquivo o sistema de opcoes e jogadores
int load_config_file(const char *filename)
{
    FIL file;
    FRESULT fr;
    char line[128];

    fr = f_open(&file, filename, FA_READ);
    if (fr != FR_OK) return -1;

    int section = 0; // 0=none, 1=options, 2=players

    while (f_gets(line, sizeof(line), &file)) {

        line[strcspn(line, "\r\n")] = 0;

        if (line[0] == 0) continue;

        if (strncmp(line, "OPTIONS=", 8) == 0) {
            num_options = atoi(line + 8);
            continue;
        }
        if (strncmp(line, "PLAYERS=", 8) == 0) {
            // Nao utilizado no final
            continue;
        }

        // Determina a secao atual do arquivo, nao eh ideal, mas para o MVP funciona
        if (isdigit(line[0])) {
            section = 1;
        } 
        else if (isxdigit(line[0])) {
            section = 2;
        }

        if (section == 1) {
            int id;
            char name[32];
            char formula[32];
            int type;

            // formato: id;name;formula;type
            sscanf(line, "%d;%31[^;];%31[^;];%d", 
                &id, name, formula, &type);

            options[id-1].id = id;
            strcpy(options[id-1].name, name);
            strcpy(options[id-1].formula, formula);
            options[id-1].type = type;
        }

        else if (section == 2) {
            // Formato:
            // UID;KEY=VAL;KEY=VAL;KEY=VAL...
            PlayerData pd = {0};

            char *tok = strtok(line, ";");
            strcpy(pd.uid, tok);

            while ((tok = strtok(NULL, ";"))) {
                char key[16], val[16];
                sscanf(tok, "%15[^=]=%15s", key, val);

                if (strcmp(key, "NAME") == 0) strcpy(pd.name, val);
                else if (strcmp(key, "STR") == 0) pd.attr[0] = atoi(val);
                else if (strcmp(key, "DEX") == 0) pd.attr[1] = atoi(val);
                else if (strcmp(key, "CON") == 0) pd.attr[2] = atoi(val);
                else if (strcmp(key, "INT") == 0) pd.attr[3] = atoi(val);
                else if (strcmp(key, "WIS") == 0) pd.attr[4] = atoi(val);
                else if (strcmp(key, "CHA") == 0) pd.attr[5] = atoi(val);
                else if (strcmp(key, "AC") == 0) pd.ac = atoi(val);
            }

            // store
            players[num_players++] = pd;
        }
    }

    sprintf(line, "Config loaded: %d options, %d players\r\n", num_options, num_players);
    uart_send_string(line);

    return 0;
}

// Pega a formula e encontra o dado a ser rolado
int parse_formula(const char *str, PlayerData *p)
{
    int n = 1, d = 20, mod = 0;

    // basic NdX
    sscanf(str, "%dd%d", &n, &d);

    // find +STAT
    const char *plus = strchr(str, '+');
    if (plus) {
        if (isdigit(plus[1])) {
            mod = atoi(plus + 1);
        } else {
            // stat name
            mod = get_player_stat(p, plus + 1);
        }
    }

    return roll(n, d) + mod;
}

// Obtem o valor de um atributo do jogador, fixo para o unico sistema no momento
int get_player_stat(PlayerData *p, const char *stat)
{
    if (strcmp(stat, "STR") == 0) return p->attr[0];
    if (strcmp(stat, "DEX") == 0) return p->attr[1];
    if (strcmp(stat, "CON") == 0) return p->attr[2];
    if (strcmp(stat, "INT") == 0) return p->attr[3];
    if (strcmp(stat, "WIS") == 0) return p->attr[4];
    if (strcmp(stat, "CHA") == 0) return p->attr[5];
    return 0;
}

// Interrupcao do Timer1 para o cartao SD
ISR(TIMER1_COMPA_vect) {
    mmc_disk_timerproc();
}

// Interrupcao de movimento do MPU6050
ISR(INT4_vect) {
    motionDetected = 1;
}