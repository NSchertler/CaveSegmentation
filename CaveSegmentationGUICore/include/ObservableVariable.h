#pragma once

#include "cavesegmentationguicore_global.h"

#include <QObject>
#include <QSlider>
#include <QLabel>

class CAVESEGMENTATIONGUICORE_EXPORT ObservableVariableBase : public QObject
{
	Q_OBJECT
public:
	ObservableVariableBase(QObject* parent = 0) : QObject(parent)
	{ }
signals:
	void changed();
};

template <typename T>
class ObservableVariable : public ObservableVariableBase
{
public:
	ObservableVariable(T& var)
		: var(var), sld(nullptr), lbl(nullptr)
	{ }

	void set(T v) { var = v; emit changed(); }
	const T& get() const { return var; }

	void setupSlider(QSlider* sld, T min, T max, int steps)
	{
		this->sld = sld;
		sliderValueStepLength = (max - min) / steps;
		sld->setMinimum(min / sliderValueStepLength);
		sld->setMaximum(max / sliderValueStepLength);
		sld->setValue(get() / sliderValueStepLength);

		connect(sld, &QSlider::valueChanged, this, &ObservableVariable<T>::sldChanged);
	}

	void setupLabel(QLabel* lbl)
	{
		this->lbl = lbl;
		lbl->setText(QString::number(get(), 'f', 2));
		connect(this, &ObservableVariable<T>::changed, this, &ObservableVariable<T>::valueChanged);
	}

private:
	void sldChanged(int value)
	{
		T realValue = value * sliderValueStepLength;
		set(realValue);
	}

	void valueChanged()
	{
		if (sld)
			sld->setValue(get() / sliderValueStepLength);
		if(lbl)
			lbl->setText(QString::number(get(), 'f', 2));
	}

private:
	T& var;

	QLabel* lbl;
	QSlider* sld;
	T sliderValueStepLength;
};