#include "gui.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

/**
 * @brief Display the welcome screen on the SSD1306 display
 */
void GUI_Welcome(void) {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(20, 5);
    ssd1306_WriteString("Welcome", Font_16x26, White);
    ssd1306_DrawBitmap(50, 35, locked, 30, 30, White);
    ssd1306_UpdateScreen();
}

/**
 * @brief Initialize the GUI by initializing the SSD1306 display and showing the welcome screen
 */
void GUI_init(void) {
    ssd1306_Init();
    GUI_Welcome();
}

/**
 * @brief Display a 'Fail' message on the SSD1306 display
 */
void GUI_Fail(void) {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(20, 5);
    ssd1306_WriteString("Fail", Font_16x26, White);
    ssd1306_DrawBitmap(50, 35, locked, 30, 30, White);
    ssd1306_UpdateScreen();
}

/**
 * @brief Display a 'Locked' message on the SSD1306 display
 */
void GUI_locked(void) {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(20, 5);
    ssd1306_WriteString("Locked", Font_16x26, White);
    ssd1306_DrawBitmap(50, 35, locked, 30, 30, White);
    ssd1306_UpdateScreen();
}

/**
 * @brief Display an 'Unlocked' message on the SSD1306 display
 */
void GUI_unlocked(void) {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(5, 5);
    ssd1306_WriteString("Unlocked", Font_16x26, White);
    ssd1306_UpdateScreen();
}

/**
 * @brief Display the initial message for updating the password on the SSD1306 display
 */
void GUI_update_password_init(void) {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(5, 5);
    ssd1306_WriteString("New PW:", Font_16x26, White);
    ssd1306_UpdateScreen();
}

/**
 * @brief Display the updated password on the SSD1306 display
 * @param password Pointer to the password string to be displayed
 */
void GUI_update_password(uint8_t *password) {
    ssd1306_SetCursor(10, 35);
    ssd1306_WriteString((char *)password, Font_7x10, White);
    ssd1306_UpdateScreen();
}

/**
 * @brief Display a 'Success' message for the password update on the SSD1306 display
 */
void GUI_update_password_success(void) {
    ssd1306_SetCursor(5, 35);
    ssd1306_WriteString("Success!", Font_16x26, White);
    ssd1306_UpdateScreen();
}
