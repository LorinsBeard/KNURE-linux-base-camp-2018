/*
 * GL BaseCamp MFRC522 RFID driver API
 * Copyright (C) 2018 Oleksii Klochko <lorins.dm@gmail.com>
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#ifndef MFRC522_API_H
#define MFRC522_API_H
int isCardPresent(uint32_t *uid);

#endif //MFRC522_API_H