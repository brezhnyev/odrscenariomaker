#ifndef GLOBALS_H
#define GLOBALS_H

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

#endif