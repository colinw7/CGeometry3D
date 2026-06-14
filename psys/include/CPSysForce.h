#ifndef CPSysForce_H
#define CPSysForce_H

class CPSysForce {
 public:
  virtual ~CPSysForce() { }

  virtual void turnOn () = 0;
  virtual void turnOff() = 0;

  virtual bool isOn () const = 0;
  virtual bool isOff() const = 0;

  virtual void apply() = 0;
};

#endif
