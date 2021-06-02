#ifndef VECTORCIRCULAR_H
#define VECTORCIRCULAR_H

#include <vector>

#include <QDebug>

template<typename  T>
class VectorCircular
{
public:
    VectorCircular(): _first(nullptr), _last(nullptr), _size(0) {}

    class Iterator: public std::iterator<std::input_iterator_tag, T>
    {
    public:
        friend VectorCircular;
        Iterator(T * t=nullptr, VectorCircular *parent=nullptr): _t(t), _parent(parent) {  }
        Iterator(const Iterator &it): _t(it._t), _parent(it._parent) {  }

        bool operator!=(const Iterator &o)  {
            if ( idx()==_parent->_size+1 && o.idx()==-1) { return false; }
            return _t!=o._t;
        }
        bool operator==(const Iterator &o) const { return !(*this!=o); }
        Iterator& operator=(const Iterator &o) { _t = o._t; _parent = o._parent; return *this; }


        typename Iterator::reference operator*() const  { return *(_parent->_first+key()); }

        T* operator->() { return _parent->_first+key(); }

        Iterator &operator++() {
            int64_t i = idx();
            if ( i==-1 ) { i = 0; } else
            if ( i==_parent->_size+1 ) { i = -1; }
            _t = _parent->_first + i +1;

            return *this;
        }

        Iterator operator+(size_t n) {
            Iterator it(*this);
            int64_t i = idx();


           /* if ( i==-1 ) { i=0; }
            size_t offset = (i+n)%(_parent->_size);
            it._t = _parent->_first + offset;
            return  it;*/
        }

        int64_t idx() const { return (_t-_parent->_first); }
        size_t key() const {
            int64_t i = idx();
            if ( i==-1 ) { return 0; }
            return i%_parent->_size;
        }

    private:
        T * _t;
        VectorCircular * _parent;        
    };

    void reserve(size_t n) { _v.reserve(n); }

    void push_back(const T &v) {
        _v.push_back(v);
        _first = &_v[0];
        _last = &_v[_size];
        _size++;
    }

    T &operator[](size_t idx) { return _v[idx%_size]; }
    const T &operator[](size_t idx) const { return _v[idx%_size]; }
    T &at(size_t idx) { return _v.at(idx%_size); }
    const T &at(size_t idx) const { return _v.at(idx%_size); }

    size_t size() const { return _size; }

    Iterator erase(const Iterator &it) {
        _v.erase(_v.begin()+it.key());
        _size--;
        _first = (_size>0)? &_v[0] : nullptr;
        _last = (_size>0)? &_v[_size-1] : nullptr;
        return Iterator(_first, this);
    }

    T &fisrt() { return _first; }
    T &last() { return _last; }

    Iterator begin() { return Iterator(_first-1, this); }
    Iterator end() { return Iterator(_last+1, this); }

private:
    T * _first;
    T * _last;
    size_t _size;
    std::vector<T> _v; //--  b 0 1 2 3 4 5 e b


};

#endif // VECTORCIRCULAR_H
