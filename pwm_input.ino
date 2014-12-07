double readPWM(int pin) {
    double h = pulseIn(pin, HIGH);
    double l = pulseIn(pin, LOW);
    return h / (h + l);
}
