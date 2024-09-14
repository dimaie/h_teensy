#ifndef SI570_h
#define SI570_h

class SI570 {
  public:
    SI570(bool _init = false);
    set_frequency(unsigned long frequency);
    init();
};

#endif