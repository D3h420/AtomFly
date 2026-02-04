# AtomFly Web Test (minimal)

Minimalny, testowy projekt do sterowania M5Stack Atom Fly z przeglądarki przez Wi‑Fi AP.

## Zawartość
- `AtomFlyWebTest/AtomFlyWebTest.ino`
- `AtomFlyWebTest/AtomFly.h`
- `AtomFlyWebTest/AtomFly.cpp`

Pliki `AtomFly.h/.cpp` pochodzą z oficjalnych przykładów M5Stack i są wymagane do użycia oficjalnych API Atom Fly.

## Wymagane biblioteki Arduino
- `M5Atom`
- `Adafruit BMP280`

## Flashowanie
1. Otwórz `AtomFlyWebTest/AtomFlyWebTest.ino` w Arduino IDE.
2. Zainstaluj biblioteki: `M5Atom` i `Adafruit BMP280`.
3. Wybierz odpowiednią płytkę (M5Stack Atom / ESP32) i port.
4. Wgraj firmware.

## Połączenie
- SSID: `AtomFly-Test`
- Hasło: `atomfly123`
- Strona: `http://192.168.4.1`

## Komendy (UI)
- `START` — uruchamia silniki (ustawia bazowe PWM).
- `UP` — zwiększa ciąg (podnosi drona).
- `DOWN` — zmniejsza ciąg (opuszcza drona); gdy PWM spadnie do 0, dron się rozbraja.
- `PRZÓD` — krótki „popych” do przodu (czasowo zmienia balans).
- `OBRÓT 90° W LEWO` — czasowy obrót w lewo.
- `OBRÓT 90° W PRAWO` — czasowy obrót w prawo.
- `🚨 EMERGENCY STOP 🚨` — natychmiast odcina silniki i rozbraja (działa też na przycisku urządzenia).

## Bezpieczeństwo (ważne)
- Sterowanie po Wi‑Fi nie daje 100% gwarancji (zakłócenia RF, opóźnienia, zawieszenie).  
  Zawsze miej fizyczne odcięcie zasilania w zasięgu ręki.
- `EMERGENCY STOP` natychmiast odcina silniki i rozbraja, ale sieć może zawieść.
- Parametry w kodzie są celowo konserwatywne. Zwiększaj je stopniowo.

## Uwaga
To jest prototyp testowy. Nie ma stabilizacji PID ani zaawansowanej kontroli lotu.
