#include "ejemplo1.h"

//QTimer *timer2;

Timer mytimer2;

ejemplo1::ejemplo1() : Ui_Counter()
{
	setupUi(this);
	show();

	//timer2 = new QTimer(this);

	
	horizontalSlider->setMaximum(2000);
	horizontalSlider->setMinimum(100);

	lcdNumber_3->display(horizontalSlider->value());
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()));
	connect(horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(changePeriod()));
	//connect(timer2, SIGNAL(timeout()), this, SLOT(moreTime()));
	//timer2->start(1000);
	mytimer.connect(std::bind(&ejemplo1::cuenta, this));
	mytimer.start(horizontalSlider->value());
	mytimer2.connect(std::bind(&ejemplo1::moreTime, this));
	mytimer2.start(1000);
}

ejemplo1::~ejemplo1()
{
}

void ejemplo1::doButton()
{
	static bool stopped = false;
	stopped = !stopped;
	if (stopped)
		mytimer.stop();
	else
		mytimer.start(horizontalSlider->value());
	qDebug() << "click on button";
}

void ejemplo1::moreTime()
{
	lcdNumber_2->display(lcdNumber_2->intValue() + 1);
}

void ejemplo1::changePeriod()
{
	mytimer.start(horizontalSlider->value());
	lcdNumber_3->display(horizontalSlider->value());
}

void ejemplo1::cuenta()
{
	lcdNumber->display(++cont);
	trick++;
}
