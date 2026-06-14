#ifndef CArrayList_H
#define CArrayList_H

#include <vector>

typedef unsigned int uint;

template<typename T>
class CArrayList {
 public:
  class Iterator {
   public:
    Iterator(CArrayList &l) :
     l_(l), pos_(0) {
    }

    bool hasNext() const {
      return pos_ < int(l_.size());
    }

    T &next() {
      return l_.get(pos_++);
    }

    void remove() {
      l_.remove(pos_);
    }

   private:
    CArrayList &l_;
    int         pos_;
  };

 private:
  std::vector<T> data_;

 public:
  CArrayList() { }

 ~CArrayList() { }

  void add(const T &t) {
    data_.push_back(t);
  }

  void addAll(const CArrayList &a) {
    uint num = a.size();

    for (uint i = 0; i < num; ++i)
      add(a.get(i));
  }

  uint size() const {
    return uint(data_.size());
  }

  const T &operator[](int ind) const {
    return data_[uint(ind)];
  }

  T &operator[](int ind) {
    return data_[uint(ind)];
  }

  const T &get(int ind) const {
    return data_[uint(ind)];
  }

  T &get(int ind) {
    return data_[uint(ind)];
  }

  void set(int ind, const T &t) {
    data_[uint(ind)] = t;
  }

  void clear() {
    data_.clear();
  }

  T &remove(int i) {
    T &t = data_[uint(i)];

    for (int i1 = i + 1; i1 < int(size()); ++i1)
      data_[uint(i1 - 1)] = data_[uint(i1)];

    return t;
  }

  T &remove(T &t) {
    for (uint i = 0; i < size(); ++i)
      if (data_[i] == t)
        return remove(int(i));

    return data_[0];
  }

  bool isEmpty() const {
    return data_.empty();
  }

  T &lastElement() {
    return data_.back();
  }

  Iterator iterator() {
    return Iterator(*this);
  }
};

#endif
