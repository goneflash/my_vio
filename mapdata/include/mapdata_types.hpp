#ifndef MAPDATA_TYPES_HPP_
#define MAPDATA_TYPES_HPP_

namespace vio {


template <typename T>
T CreateNewId() {
  return T();
}

// Note: Copy constructor will have the same id.
class UniqueId {
 public:
  // TODO: Maybe a random number?
  UniqueId() : id_(++unique_id_) {}

  int id() { return id_; }
 private:
  int id_;
  static int unique_id_;
};

// TODO: Learn how to not duplicate the code.
// Note: Copy constructor will have the same id.
class KeyframeId {
 public:
  KeyframeId() : id_(++unique_id_) {}

  int id() { return id_; }
 private:
  int id_;
  static int unique_id_;
};

class LandmarkId {
 public:
  LandmarkId() : id_(++unique_id_) {}

  int id() { return id_; }
 private:
  int id_;
  static int unique_id_;
};

}  // vio

#endif
