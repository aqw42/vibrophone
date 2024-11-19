
// Potentiometer filter class
class PotFilter {
  private:
    static const int FILTER_SAMPLES = 8;  // Adjust this value as needed
    int readings[FILTER_SAMPLES];
    int index = 0;
    int pin;
    
  public:
    PotFilter(int analogPin) : pin(analogPin) {
      // Initialize all readings to 0
      for(int i = 0; i < FILTER_SAMPLES; i++) {
        readings[i] = 0;
      }
    }
    
    unsigned int read() {
      // Add new reading to array
      readings[index] = analogRead(pin);
      index = (index + 1) % FILTER_SAMPLES;
      
      // Calculate average
      long sum = 0;
      for(int i = 0; i < FILTER_SAMPLES; i++) {
        sum += readings[i];
      }
      return sum / FILTER_SAMPLES;
    }
};