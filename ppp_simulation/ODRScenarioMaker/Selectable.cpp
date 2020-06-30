#include "Selectable.h"

int Selectable::s_ID = 0;

Selectable::Selectable() : m_selected(false) 
{ 
    ++s_ID; m_id = s_ID;
}