typedef struct {
    //char c;
    //char dummy[3];
    int32_t x;
    int32_t y;
} TData;

void setup() {
    Serial.begin(9600);
}

void loop() {
    if (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == 's') {
            TData d;
            //d.c = 'a';
            d.x = 100;
            d.y = 200;
            Serial.write((uint8_t)sizeof(d));          // first byte: size
            Serial.write((uint8_t*)&d, sizeof(d));     // the struct itself
        }
    }
}
