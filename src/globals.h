#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>

// add member
#define ADDVAR(access, type, var, initVal)\
    access: type m_##var{initVal};\
    public:\
    void set_##var(type val) { m_##var = val; }\
    type get_##var() const { return m_##var; }

// add member, return reference
#define ADDVRR(access, type, var, initVal)\
    access: type m_##var{initVal};\
    public:\
    void set_##var(type val) { m_##var = val; }\
    type & get_##var() { return m_##var; }

// used as workaround in parent() function see treemodel.cpp
constexpr uint64_t MAGICNUMBER {1176543210987654320LL};

#endif
