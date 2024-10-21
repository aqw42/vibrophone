#include "Blue.h"

Blue::Blue(String n)
  : name(n), a2dp_sink(BluetoothA2DPSink(out)) {}

Blue::Blue(Blue &&other)
  : name(other.name), a2dp_sink(other.a2dp_sink) {}

void Blue::enable() {
  a2dp_sink.start(name.c_str(), true);
}

void Blue::disable() {
  a2dp_sink.end();
}

void Blue::update() {
}

void Blue::display() {

}