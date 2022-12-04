#include <QDoubleSpinBox>
#include <QLabel>

class LDoubleSpinBox : public QWidget
{
  Q_OBJECT
public:
  LDoubleSpinBox(QWidget * parent, float value, float minval, float maxval, float step, std::string label);
  double value() { return m_spinBox->value(); }
  void setValue(double val) { m_spinBox->setValue(val); }

signals:
  void valueChanged(double);

public:
  QDoubleSpinBox  * m_spinBox;
  QLabel          * m_label;
};