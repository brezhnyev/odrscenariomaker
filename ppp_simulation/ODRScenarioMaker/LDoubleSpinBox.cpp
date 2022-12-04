#include "LDoubleSpinBox.h"
#include <QHBoxLayout>

LDoubleSpinBox::LDoubleSpinBox(QWidget * parent, float val, float minval, float maxval, float step, std::string label) : QWidget(parent)
{
    QHBoxLayout * l = new QHBoxLayout();
    m_spinBox = new QDoubleSpinBox(this);
    m_spinBox->setRange(minval, maxval);
    m_spinBox->setSingleStep(step);
    m_spinBox->setValue(val);
    l->addWidget(m_label = new QLabel(label.c_str()));
    l->addWidget(m_spinBox);
    setLayout(l);

    connect(m_spinBox, SIGNAL(valueChanged(double)), this, SIGNAL(valueChanged(double)));
}