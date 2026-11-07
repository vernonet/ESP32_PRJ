import re

with open("src/data/esp32_JK_BMS_original.cpp", "r", encoding="utf-8") as f:
    lines = f.readlines()

new_lines = []
for line in lines:
    l = line
    # Header
    if "* Скетч ESP32 для читання даних з Jikong (JK) BMS по BLE (Bluetooth Low Energy)" in l:
        l = l.replace("* Скетч ESP32 для читання даних з Jikong (JK) BMS по BLE (Bluetooth Low Energy)", "* ESP32 sketch for reading data from Jikong (JK) BMS over BLE (Bluetooth Low Energy)")
    if "* ESP32 виступає в ролі BLE Client." in l:
        l = l.replace("* ESP32 виступає в ролі BLE Client.", "* ESP32 operates as a BLE Client.")
    if "* ОПТИМІЗОВАНО: Переведено на AJAX/JSON для живого оновлення даних та економії RAM." in l:
        l = l.replace("* ОПТИМІЗОВАНО: Переведено на AJAX/JSON для живого оновлення даних та економії RAM.", "* OPTIMIZED: Converted to AJAX/JSON for live data updates and RAM efficiency.")
    
    # Process frames
    if "// Перевірка на початок нового кадру даних (AA 55 EB 90)" in l:
        l = l.replace("// Перевірка на початок нового кадру даних (AA 55 EB 90)", "// Check for start of a new data frame (AA 55 EB 90)")
    if "// Якщо у буфері вже лежать дані попереднього кадру і їх достатньо для парсингу" in l:
        l = l.replace("// Якщо у буфері вже лежать дані попереднього кадру і їх достатньо для парсингу", "// If previous frame data is already in buffer and sufficient for parsing")
    if 'Serial.println("-> Розпарсено Info!");' in l:
        l = l.replace('Serial.println("-> Розпарсено Info!");', 'Serial.println("-> Parsed Info!");')
    if 'Serial.println("-> Розпарсено Settings!");' in l:
        l = l.replace('Serial.println("-> Розпарсено Settings!");', 'Serial.println("-> Parsed Settings!");')
    if 'Serial.printf("-> Розпарсено Telemetry! (Отримано %d байт)\\n", frame);' in l:
        l = l.replace('Serial.printf("-> Розпарсено Telemetry! (Отримано %d байт)\\n", frame);', 'Serial.printf("-> Parsed Telemetry! (Received %d bytes)\\n", frame);')
    if "// Скидаємо лічильник для накопичення нового кадру" in l:
        l = l.replace("// Скидаємо лічильник для накопичення нового кадру", "// Reset counter to accumulate new frame")
    if "// Накопичуємо шматочки одного й того самого кадру в буфер" in l:
        l = l.replace("// Накопичуємо шматочки одного й того самого кадру в буфер", "// Accumulate frame chunks into buffer")
    if "// Оскільки пакети телеметрії Jikong довгі (близько 300 байт)," in l:
        l = l.replace("// Оскільки пакети телеметрії Jikong довгі (близько 300 байт),", "// Because Jikong telemetry packets are long (~300 bytes),")
    if "// якщо ми вже набрали повний пакет, можемо розпарсити його превентивно, не чекаючи наступного заголовка" in l:
        l = l.replace("// якщо ми вже набрали повний пакет, можемо розпарсити його превентивно, не чекаючи наступного заголовка", "// if full packet is collected, parse it preemptively without waiting for next header")

    # BLE low level
    if "// Включаємо сповіщення (Notify) — оптимізовано під Jikong BMS" in l:
        l = l.replace("// Включаємо сповіщення (Notify) — оптимізовано під Jikong BMS", "// Enable notifications (Notify) — optimized for Jikong BMS")
    if "// Включаємо сповіщення (Notify) — НИЗЬКОРІВНЕВИЙ ОБХІД БАГУ ДЛЯ JIKONG BMS" in l:
        l = l.replace("// Включаємо сповіщення (Notify) — НИЗЬКОРІВНЕВИЙ ОБХІД БАГУ ДЛЯ JIKONG BMS", "// Enable notifications (Notify) — LOW-LEVEL BUG WORKAROUND FOR JIKONG BMS")
    if 'Serial.println("Реєстрація обробника сповіщень...");' in l:
        l = l.replace('Serial.println("Реєстрація обробника сповіщень...");', 'Serial.println("Registering notification callback handler...");')
    if "// false, щоб не запускати дефектний стандартний сканер дескрипторів" in l:
        l = l.replace("// false, щоб не запускати дефектний стандартний сканер дескрипторів", "// false to prevent buggy standard descriptor scanner")
    if 'Serial.println("Примусова низькорівнева підписка на CCCD (0x2902)...");' in l:
        l = l.replace('Serial.println("Примусова низькорівнева підписка на CCCD (0x2902)...");', 'Serial.println("Forcing low-level subscription to CCCD (0x2902)...");')
    if "// Отримуємо внутрішні системні ідентифікатори BLE-стеку ESP32" in l:
        l = l.replace("// Отримуємо внутрішні системні ідентифікатори BLE-стеку ESP32", "// Retrieve internal ESP32 BLE stack handles")
    if "// Складаємо сирий пакет активації сповіщень (Little-Endian: 0x0001)" in l:
        l = l.replace("// Складаємо сирий пакет активації сповіщень (Little-Endian: 0x0001)", "// Assemble raw notification enable packet (Little-Endian: 0x0001)")
    if "// Викликаємо пряму функцію ядра ESP-IDF для запису в дескриптор (Handle характеристики + 1)" in l:
        l = l.replace("// Викликаємо пряму функцію ядра ESP-IDF для запису в дескриптор (Handle характеристики + 1)", "// Call ESP-IDF core function directly to write descriptor (char handle + 1)")
    if "// Хандл дескриптора CCCD у плат Jikong завжди дорівнює Handle характеристики + 1" in l:
        l = l.replace("// Хандл дескриптора CCCD у плат Jikong завжди дорівнює Handle характеристики + 1", "// CCCD descriptor handle on Jikong boards is always char handle + 1")
    if 'Serial.println("Підписка на рівні ядра завершена успішно!");' in l:
        l = l.replace('Serial.println("Підписка на рівні ядра завершена успішно!");', 'Serial.println("Kernel-level CCCD subscription successful!");')
    if 'Serial.printf("Помилка низькорівневого запису: 0x%X\\n", err);' in l:
        l = l.replace('Serial.printf("Помилка низькорівневого запису: 0x%X\\n", err);', 'Serial.printf("Low-level descriptor write error: 0x%X\\n", err);')

    # Web pages - handleRoot
    if "<title>BMS ВІДКЛЮЧЕНО</title>" in l:
        l = l.replace("<title>BMS ВІДКЛЮЧЕНО</title>", "<title>BMS DISCONNECTED</title>")
    if "<h2>⚡️ Статус BMS: ВІДКЛЮЧЕНО</h2>" in l:
        l = l.replace("<h2>⚡️ Статус BMS: ВІДКЛЮЧЕНО</h2>", "<h2>⚡️ BMS Status: DISCONNECTED</h2>")
    if "// ДОДАНО: Кнопка OTA на екран відключеного стану (як в оригіналі)" in l:
        l = l.replace("// ДОДАНО: Кнопка OTA на екран відключеного стану (як в оригіналі)", "// ADDED: OTA button on disconnected screen (as in original)")
    if "return confirm('Перезавантажити ESP32?');" in l:
        l = l.replace("return confirm('Перезавантажити ESP32?');", "return confirm('Reboot ESP32?');")
    if ">🔍 СКАНУВАТИ ТА ВИБРАТИ BMS</a>" in l:
        l = l.replace(">🔍 СКАНУВАТИ ТА ВИБРАТИ BMS</a>", ">🔍 SCAN & SELECT BMS</a>")
    if "<title>Дані Jikong BMS</title>" in l:
        l = l.replace("<title>Дані Jikong BMS</title>", "<title>Jikong BMS Data</title>")
    if "<h2>⚡️ Живі дані Jikong BMS</h2>" in l:
        l = l.replace("<h2>⚡️ Живі дані Jikong BMS</h2>", "<h2>⚡️ Live Jikong BMS Telemetry</h2>")
    if "// ВІДНОВЛЕНО ТА ПОКРАЩЕНО: Красиве текстове меню з кнопкою OTA на головному екрані" in l:
        l = l.replace("// ВІДНОВЛЕНО ТА ПОКРАЩЕНО: Красиве текстове меню з кнопкою OTA на головному екрані", "// ENHANCED: Clean navigation menu with OTA button on dashboard")
    if "<a href='/settings' class='btn'>⚙️ Налаштування</a>" in l:
        l = l.replace("<a href='/settings' class='btn'>⚙️ Налаштування</a>", "<a href='/settings' class='btn'>⚙️ Settings</a>")
    if "<a href='/info' class='btn' style='background:#17a2b8'>ℹ️ Інфо</a>" in l:
        l = l.replace("<a href='/info' class='btn' style='background:#17a2b8'>ℹ️ Інфо</a>", "<a href='/info' class='btn' style='background:#17a2b8'>ℹ️ Info</a>")
    if "<a href='/disconnect' class='btn off'>❌ Вимкнути BLE</a>" in l:
        l = l.replace("<a href='/disconnect' class='btn off'>❌ Вимкнути BLE</a>", "<a href='/disconnect' class='btn off'>❌ Disconnect BLE</a>")
    if "<tr><th>Параметр</th><th>Значення</th><th>Од.</th></tr>" in l:
        l = l.replace("<tr><th>Параметр</th><th>Значення</th><th>Од.</th></tr>", "<tr><th>Parameter</th><th>Value</th><th>Unit</th></tr>")
    if "<tr><td>Напруга батареї</td>" in l:
        l = l.replace("<tr><td>Напруга батареї</td>", "<tr><td>Battery Voltage</td>")
    if "<tr><td>Струм заряду/розряду</td>" in l:
        l = l.replace("<tr><td>Струм заряду/розряду</td>", "<tr><td>Charge / Discharge Current</td>")
    if "<tr><td>Потужність</td>" in l:
        l = l.replace("<tr><td>Потужність</td>", "<tr><td>Power</td>")
    if "<tr><td>Залишок заряду (SOC)</td>" in l:
        l = l.replace("<tr><td>Залишок заряду (SOC)</td>", "<tr><td>State of Charge (SOC)</td>")
    if "<tr><td>Дельта комірок</td>" in l:
        l = l.replace("<tr><td>Дельта комірок</td>", "<tr><td>Cell Delta</td>")
    if "<tr><td>Температура T1 / T2</td>" in l:
        l = l.replace("<tr><td>Температура T1 / T2</td>", "<tr><td>Temperature T1 / T2</td>")
    if "<tr><td>Температура MOS</td>" in l:
        l = l.replace("<tr><td>Температура MOS</td>", "<tr><td>MOSFET Temperature</td>")
    if "<tr><td>Заряд</td>" in l:
        l = l.replace("<tr><td>Заряд</td>", "<tr><td>Charge Switch</td>")
    if "<tr><td>Розряд</td>" in l:
        l = l.replace("<tr><td>Розряд</td>", "<tr><td>Discharge Switch</td>")
    if "<tr><td>Баланс</td>" in l:
        l = l.replace("<tr><td>Баланс</td>", "<tr><td>Balance Switch</td>")
    if "<h2>🔬 Напруги комірок</h2>" in l:
        l = l.replace("<h2>🔬 Напруги комірок</h2>", "<h2>🔬 Cell Voltages</h2>")

    # JS updates in handleRoot
    if "s_chg" in l and "УВІМК" in l:
        l = '  html += "    if(document.getElementById(\'s_chg\')) document.getElementById(\'s_chg\').innerHTML = d.chg ? \'<span style=\\\"color:green\\\">🟢 ON</span>\' : \'<span style=\\\"color:red\\\">🔴 OFF</span>\';";\n'
    if "b_chg" in l and "ВИМК" in l:
        l = '  html += "    if(document.getElementById(\'b_chg\')) document.getElementById(\'b_chg\').innerHTML = d.chg ? \'<a href=\\\"#\\\" onclick=\\\"fetch(\\\' /charge_off\\\'); return false;\\\" class=\\\"btn off\\\">TURN OFF</a>\' : \'<a href=\\\"#\\\" onclick=\\\"fetch(\\\' /charge_on\\\'); return false;\\\" class=\\\"btn on\\\">TURN ON</a>\';";\n'
        l = l.replace(" /charge_off", "/charge_off").replace(" /charge_on", "/charge_on")
    if "s_dsg" in l and "УВІМК" in l:
        l = '  html += "    if(document.getElementById(\'s_dsg\')) document.getElementById(\'s_dsg\').innerHTML = d.dsg ? \'<span style=\\\"color:green\\\">🟢 ON</span>\' : \'<span style=\\\"color:red\\\">🔴 OFF</span>\';";\n'
    if "b_dsg" in l and "ВИМК" in l:
        l = '  html += "    if(document.getElementById(\'b_dsg\')) document.getElementById(\'b_dsg\').innerHTML = d.dsg ? \'<a href=\\\"#\\\" onclick=\\\"fetch(\\\' /discharge_off\\\'); return false;\\\" class=\\\"btn off\\\">TURN OFF</a>\' : \'<a href=\\\"#\\\" onclick=\\\"fetch(\\\' /discharge_on\\\'); return false;\\\" class=\\\"btn on\\\">TURN ON</a>\';";\n'
        l = l.replace(" /discharge_off", "/discharge_off").replace(" /discharge_on", "/discharge_on")
    if "s_bal" in l and "Активний" in l:
        l = '        document.getElementById(\'s_bal\').innerHTML = \'<span style=\\\"color:green\\\">🟢 Active</span> (\' + bc.toFixed(3) + \' A) <a href=\\\"#\\\" onclick=\\\"fetch(\\\' /balance_off\\\'); return false;\\\" class=\\\"btn off\\\">TURN OFF</a>\';\n'
        l = l.replace(" /balance_off", "/balance_off")
    if "s_bal" in l and "⚪ Ні" in l:
        l = '        document.getElementById(\'s_bal\').innerHTML = \'⚪ Inactive <a href=\\\"#\\\" onclick=\\\"fetch(\\\' /balance_on\\\'); return false;\\\" class=\\\"btn on\\\">TURN ON</a>\';\n'
        l = l.replace(" /balance_on", "/balance_on")
    if "<tr><th>Комірка</th><th>Напруга (V)</th><th>Опір (Ом)</th></tr>" in l:
        l = l.replace("<tr><th>Комірка</th><th>Напруга (V)</th><th>Опір (Ом)</th></tr>", "<tr><th>Cell</th><th>Voltage (V)</th><th>Resistance (Ω)</th></tr>")

    # handleInfo
    if "<h2>ℹ️ Інформація</h2><table><tr><th>Параметр</th><th>Значення</th></tr>" in l:
        l = l.replace("<h2>ℹ️ Інформація</h2><table><tr><th>Параметр</th><th>Значення</th></tr>", "<h2>ℹ️ Device Information</h2><table><tr><th>Parameter</th><th>Value</th></tr>")
    if 'addR("Пристрій",' in l:
        l = l.replace('addR("Пристрій",', 'addR("Device Name",')
    if 'addR("Серійний №",' in l:
        l = l.replace('addR("Серійний №",', 'addR("Serial Number",')
    if 'addR("Версія HW/SW",' in l:
        l = l.replace('addR("Версія HW/SW",', 'addR("HW / SW Version",')
    if 'addR("Компіляція",' in l:
        l = l.replace('addR("Компіляція",', 'addR("Firmware Build Time",')
    if 'addR("Дата виготовлення",' in l:
        l = l.replace('addR("Дата виготовлення",', 'addR("Manufacturing Date",')
    if 'addR("Користувач",' in l:
        l = l.replace('addR("Користувач",', 'addR("User Custom Data",')
    if "<a href='/' class='btn'>◀️ ГОЛОВНА</a>" in l:
        l = l.replace("<a href='/' class='btn'>◀️ ГОЛОВНА</a>", "<a href='/' class='btn'>◀️ DASHBOARD</a>")

    # handleSettings
    if "// Повідомляємо браузеру про передачу даних частинами" in l:
        l = l.replace("// Повідомляємо браузеру про передачу даних частинами", "// Notify browser about chunked transfer encoding")
    if "// --- ЧАНК 1: ШАПКА ТА СТИЛІ ---" in l:
        l = l.replace("// --- ЧАНК 1: ШАПКА ТА СТИЛІ ---", "// --- CHUNK 1: HEADER & STYLES ---")
    if "<title>Налаштування Jikong BMS</title>" in l:
        l = l.replace("<title>Налаштування Jikong BMS</title>", "<title>Jikong BMS Settings</title>")
    if "<h2>⚙️ Налаштування Jikong BMS</h2><a href='/'>⬅️ На головну</a><br><br><form method='POST' action='/settings_update'><table><tr><th>Параметр</th><th>Поточне</th><th>Нове</th></tr>" in l:
        l = l.replace("<h2>⚙️ Налаштування Jikong BMS</h2><a href='/'>⬅️ На головну</a><br><br><form method='POST' action='/settings_update'><table><tr><th>Параметр</th><th>Поточне</th><th>Нове</th></tr>", "<h2>⚙️ Jikong BMS Settings</h2><a href='/'>⬅️ Back to Dashboard</a><br><br><form method='POST' action='/settings_update'><table><tr><th>Parameter</th><th>Current</th><th>New</th></tr>")
    if "<tr class='group'><th colspan='3'>Параметри батареї</th></tr>" in l:
        l = l.replace("<tr class='group'><th colspan='3'>Параметри батареї</th></tr>", "<tr class='group'><th colspan='3'>Battery Parameters</th></tr>")
    if "<tr><td>Номінальна ємність (Ah)</td>" in l:
        l = l.replace("<tr><td>Номінальна ємність (Ah)</td>", "<tr><td>Nominal Capacity (Ah)</td>")
    if "<tr><td>Кількість комірок</td>" in l:
        l = l.replace("<tr><td>Кількість комірок</td>", "<tr><td>Cell Count</td>")
    if "<tr><td>Режим BMS</td>" in l:
        l = l.replace("<tr><td>Режим BMS</td>", "<tr><td>BMS Mode</td>")
    if '// Очищаємо буфер перед наступною частиною' in l:
        l = l.replace('// Очищаємо буфер перед наступною частиною', '// Clear buffer before next chunk')
    if "// --- ЧАНК 2: НАПРУГИ КОМІРОК ---" in l:
        l = l.replace("// --- ЧАНК 2: НАПРУГИ КОМІРОК ---", "// --- CHUNK 2: CELL VOLTAGES ---")
    if "<tr class='group'><th colspan='3'>Напруги комірок (V)</th></tr>" in l:
        l = l.replace("<tr class='group'><th colspan='3'>Напруги комірок (V)</th></tr>", "<tr class='group'><th colspan='3'>Cell Voltages (V)</th></tr>")
    if "<tr><td>Захист від перенапруги (ОВР)</td>" in l:
        l = l.replace("<tr><td>Захист від перенапруги (ОВР)</td>", "<tr><td>Overvoltage Protection (OVP)</td>")
    if "<tr><td>Захист від низької напруги (УВР)</td>" in l:
        l = l.replace("<tr><td>Захист від низької напруги (УВР)</td>", "<tr><td>Undervoltage Protection (UVP)</td>")
    if "<tr><td>Відновлення ОВР</td>" in l:
        l = l.replace("<tr><td>Відновлення ОВР</td>", "<tr><td>OVP Recovery Voltage</td>")
    if "<tr><td>Відновлення УВР</td>" in l:
        l = l.replace("<tr><td>Відновлення УВР</td>", "<tr><td>UVP Recovery Voltage</td>")
    if "<tr><td>Напруга вимкнення (Power Off)</td>" in l:
        l = l.replace("<tr><td>Напруга вимкнення (Power Off)</td>", "<tr><td>Power-Off Voltage</td>")
    if "<tr class='group'><th colspan='3'>Зарядні / Номінальні напруги (V)</th></tr>" in l:
        l = l.replace("<tr class='group'><th colspan='3'>Зарядні / Номінальні напруги (V)</th></tr>", "<tr class='group'><th colspan='3'>Charging / Nominal Voltages (V)</th></tr>")
    if "<tr><td>Повна напруга заряду</td>" in l:
        l = l.replace("<tr><td>Повна напруга заряду</td>", "<tr><td>Full Charge Voltage</td>")
    if "<tr><td>Номінальна напруга</td>" in l:
        l = l.replace("<tr><td>Номінальна напруга</td>", "<tr><td>Nominal Voltage</td>")
    if "<tr><td>Підтримуюча напруга заряду</td>" in l:
        l = l.replace("<tr><td>Підтримуюча напруга заряду</td>", "<tr><td>Float Charge Voltage</td>")
    if "// --- ЧАНК 3: БАЛАНСУВАННЯ ТА СТРУМИ ---" in l:
        l = l.replace("// --- ЧАНК 3: БАЛАНСУВАННЯ ТА СТРУМИ ---", "// --- CHUNK 3: BALANCING & CURRENTS ---")
    if "<tr class='group'><th colspan='3'>Балансування</th></tr>" in l:
        l = l.replace("<tr class='group'><th colspan='3'>Балансування</th></tr>", "<tr class='group'><th colspan='3'>Balancing</th></tr>")
    if "<tr><td>Напруга запуску балансування (V)</td>" in l:
        l = l.replace("<tr><td>Напруга запуску балансування (V)</td>", "<tr><td>Balance Trigger Voltage (V)</td>")
    if "<tr><td>Мінімальна напруга запуску (V)</td>" in l:
        l = l.replace("<tr><td>Мінімальна напруга запуску (V)</td>", "<tr><td>Balance Starting Voltage (V)</td>")
    if "<tr><td>Макс. струм балансування (A)</td>" in l:
        l = l.replace("<tr><td>Макс. струм балансування (A)</td>", "<tr><td>Max Balance Current (A)</td>")
    if "<tr><td>Дельта напруги для зупинки (V)</td>" in l:
        l = l.replace("<tr><td>Дельта напруги для зупинки (V)</td>", "<tr><td>Balance Delta Stop Threshold (V)</td>")
    if "<tr class='group'><th colspan='3'>СТРУМИ (А)</th></tr>" in l:
        l = l.replace("<tr class='group'><th colspan='3'>СТРУМИ (А)</th></tr>", "<tr class='group'><th colspan='3'>Current Limits (A)</th></tr>")
    if "<tr><td>Макс. струм заряду (A)</td>" in l:
        l = l.replace("<tr><td>Макс. струм заряду (A)</td>", "<tr><td>Max Charge Current (A)</td>")
    if "<tr><td>Макс. струм розряду (A)</td>" in l:
        l = l.replace("<tr><td>Макс. струм розряду (A)</td>", "<tr><td>Max Discharge Current (A)</td>")
    if "<button type='submit' class='btn'>💾 ЗБЕРЕГТИ ВСІ НАЛАШТУВАННЯ</button>" in l:
        l = l.replace("<button type='submit' class='btn'>💾 ЗБЕРЕГТИ ВСІ НАЛАШТУВАННЯ</button>", "<button type='submit' class='btn'>💾 SAVE ALL SETTINGS</button>")
    if '// Завершуємо передачу даних чанками' in l:
        l = l.replace('// Завершуємо передачу даних чанками', '// Finish chunked HTTP response')

    # Scanner
    if "<title>Сканування</title>" in l:
        l = l.replace("<title>Сканування</title>", "<title>BLE Scanner</title>")
    if "<h2>🔍 Сканування BMS</h2><table><tr><th>Ім'я пристрою</th><th>MAC</th><th>Дія</th></tr>" in l:
        l = l.replace("<h2>🔍 Сканування BMS</h2><table><tr><th>Ім'я пристрою</th><th>MAC</th><th>Дія</th></tr>", "<h2>🔍 BLE BMS Scanner</h2><table><tr><th>Device Name</th><th>MAC Address</th><th>Action</th></tr>")
    if "<tr><td colspan='3'>Пристроїв не знайдено. Оновіть сторінку.</td></tr>" in l:
        l = l.replace("<tr><td colspan='3'>Пристроїв не знайдено. Оновіть сторінку.</td></tr>", "<tr><td colspan='3'>No devices found. Refresh page to rescan.</td></tr>")
    if "class='btn'>Підключитись</a>" in l:
        l = l.replace("class='btn'>Підключитись</a>", "class='btn'>Connect</a>")
    if "<a href='/'>◀️ На головну</a>" in l:
        l = l.replace("<a href='/'>◀️ На головну</a>", "<a href='/'>◀️ Back to Dashboard</a>")
    if 'server.send(500, "text/plain; charset=UTF-8", "Помилка підключення.");' in l:
        l = l.replace('server.send(500, "text/plain; charset=UTF-8", "Помилка підключення.");', 'server.send(500, "text/plain; charset=UTF-8", "Connection error.");')

    # Update
    if "<p>Оберіть файл прошивки (.bin):</p>" in l:
        l = l.replace("<p>Оберіть файл прошивки (.bin):</p>", "<p>Select firmware binary file (.bin):</p>")
    if "<input type='file' name='firmware'><input type='submit' value='Оновити'></form>" in l:
        l = l.replace("<input type='file' name='firmware'><input type='submit' value='Оновити'></form>", "<input type='file' name='firmware'><input type='submit' value='Upload & Update'></form>")
    if "<a href='/'>&#9664; На головну</a>" in l:
        l = l.replace("<a href='/'>&#9664; На головну</a>", "<a href='/'>&#9664; Back to Dashboard</a>")
    if 'Serial.printf("Отримання файлу прошивки: %s\\n", upload.filename.c_str());' in l:
        l = l.replace('Serial.printf("Отримання файлу прошивки: %s\\n", upload.filename.c_str());', 'Serial.printf("Receiving firmware binary: %s\\n", upload.filename.c_str());')
    if "// --- ДОДАНО: Розриваємо зв'язок з BMS для вивільнення пам'яті (RAM) та стабільного OTA по WiFi ---" in l:
        l = l.replace("// --- ДОДАНО: Розриваємо зв'язок з BMS для вивільнення пам'яті (RAM) та стабільного OTA по WiFi ---", "// --- ADDED: Disconnect BLE connection to free RAM and ensure stable WiFi OTA ---")
    if 'Serial.println("OTA: Відключаємо BLE Client для очищення пам\'яті...");' in l:
        l = l.replace('Serial.println("OTA: Відключаємо BLE Client для очищення пам\'яті...");', 'Serial.println("OTA: Disconnecting BLE Client to free heap memory...");')
    if "// Коротка пауза для повного вивантаження BLE-стеку з Heap RAM" in l:
        l = l.replace("// Коротка пауза для повного вивантаження BLE-стеку з Heap RAM", "// Short pause for complete BLE stack deallocation from Heap RAM")
    if 'Serial.printf("Оновлення успішне: %u байт\\nПерезавантаження...\\n", upload.totalSize);' in l:
        l = l.replace('Serial.printf("Оновлення успішне: %u байт\\nПерезавантаження...\\n", upload.totalSize);', 'Serial.printf("Update successful: %u bytes\\nRebooting...\\n", upload.totalSize);')
    if "// ВИПРАВЛЕНО: Віддаємо красиву сторінку з таймером замість сирого тексту, щоб точно не було крякозябр" in l:
        l = l.replace("// ВИПРАВЛЕНО: Віддаємо красиву сторінку з таймером замість сирого тексту, щоб точно не було крякозябр", "// Clean HTML response with reboot countdown timer")
    if "<h2>✅ Оновлення успішне!</h2>" in l:
        l = l.replace("<h2>✅ Оновлення успішне!</h2>", "<h2>✅ Update Successful!</h2>")
    if "<p>Пристрій перезавантажується... Повернення на головну через <b id='timer'>10</b> сек.</p>" in l:
        l = l.replace("<p>Пристрій перезавантажується... Повернення на головну через <b id='timer'>10</b> сек.</p>", "<p>Device is rebooting... Returning to dashboard in <b id='timer'>10</b> sec.</p>")
    if "// Даємо 1 секунду на повну передачу сторінки у браузер перед рестартом" in l:
        l = l.replace("// Даємо 1 секунду на повну передачу сторінки у браузер перед рестартом", "// Allow 1 second for complete HTTP transmission before reboot")
    if "// ВИПРАВЛЕНО: Гарне сповіщення про помилку в UTF-8" in l:
        l = l.replace("// ВИПРАВЛЕНО: Гарне сповіщення про помилку в UTF-8", "// Clean UTF-8 error notification")
    if 'server.send(500, "text/html; charset=UTF-8", "<h2>❌ Помилка оновлення! Файл пошкоджений.</h2><a href=\'/update\'>Спробувати знову</a>");' in l:
        l = l.replace('server.send(500, "text/html; charset=UTF-8", "<h2>❌ Помилка оновлення! Файл пошкоджений.</h2><a href=\'/update\'>Спробувати знову</a>");', 'server.send(500, "text/html; charset=UTF-8", "<h2>❌ Update Failed! Firmware file corrupted.</h2><a href=\'/update\'>Try again</a>");')

    # Settings update
    if "// ВИПРАВЛЕНО: Змінено друкарську помилку з uint33_t на uint32_t" in l:
        l = l.replace("// ВИПРАВЛЕНО: Змінено друкарську помилку з uint33_t на uint32_t", "// Fixed typo: cast to uint32_t")

    # Loop
    if "// Безпечна мікро-пауза для стабільності ядра" in l:
        l = l.replace("// Безпечна мікро-пауза для стабільності ядра", "// Safe micro-delay for core stability & WDT yield")

    new_lines.append(l)

cyrillic_pattern = re.compile(r"[\u0400-\u04FF]")
remaining = []
for idx, l in enumerate(new_lines, 1):
    if cyrillic_pattern.search(l):
        remaining.append((idx, l.rstrip()))

print(f"Remaining Cyrillic lines: {len(remaining)}")
for idx, l in remaining:
    print(f"Line {idx}: {l}")

with open("src/data/esp32_JK_BMS_english.cpp", "w", encoding="utf-8") as f:
    f.writelines(new_lines)

print("Saved successfully!")
