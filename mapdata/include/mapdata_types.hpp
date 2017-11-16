#ifndef MAPDATA_TYPES_HPP_
#define MAPDATA_TYPES_HPP_

#include <functional>

namespace vio {

template <typename T>
T CreateNewId() {
  return T();
}

// Note: Copy constructor will have the same id.
class UniqueId {
 public:
  // TODO: Maybe a random number?
  UniqueId() : id_(-1) {}

  int id() const { return id_; }
  bool valid() const { return id_ != -1; }

 protected:
  int id_;
};

// TODO: Learn how to not duplicate the code.
// Note: Copy constructor will have the same id.
class KeyframeId : public UniqueId {
 public:
  KeyframeId() { id_ = ++unique_id_; }
  KeyframeId(int id) { id_ = id; }

 private:
  static int unique_id_;
};

// !!This must inline or implementation in .cpp file
inline bool operator==(const KeyframeId &id0, const KeyframeId &id1) {
  return id0.id() == id1.id();
}

// This is used for map<KeyframeId>
inline bool operator<(const KeyframeId &id0, const KeyframeId &id1) {
  return id0.id() < id1.id();
}

inline bool operator>(const KeyframeId &id0, const KeyframeId &id1) {
  return id0.id() > id1.id();
}

class LandmarkId : public UniqueId {
 public:
  LandmarkId() { id_ = ++unique_id_; }
  LandmarkId(int id) { id_ = id; }

 private:
  static int unique_id_;
};

// !!This must inline or implementation in .cpp file
inline bool operator==(const LandmarkId &id0, const LandmarkId &id1) {
  return id0.id() == id1.id();
}
}  // vio

// TODO: Is there better way?
namespace std {
// Make hash for Id.
template <>
struct hash<vio::UniqueId> {
  std::size_t operator()(const vio::UniqueId &id) const {
    return std::hash<int>()(id.id());
  }
};

template <>
struct hash<vio::KeyframeId> {
  std::size_t operator()(const vio::KeyframeId &id) const {
    return std::hash<int>()(id.id());
  }
};

template <>
struct hash<vio::LandmarkId> {
  std::size_t operator()(const vio::LandmarkId &id) const {
    return std::hash<int>()(id.id());
  }
};

}  // std

#endif
