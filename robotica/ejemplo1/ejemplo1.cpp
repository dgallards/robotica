#include "ejemplo1.h"

int counter = 0;
QTimer *timer;

ejemplo1::ejemplo1() : Ui_Counter()
{
	timer = new QTimer(this);

	//timer->setInterval(1000);

	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()));
	connect(timer, SIGNAL(timeout()), this, SLOT(doTick()));
	timer->start(1000);
}

void ejemplo1::doButton()
{
	counter = 0;
	this->lcdNumber->display(counter);
	this->lcdNumber->update();
	timer->start();

	qDebug() << "click on button";
}

void ejemplo1::doTick()
{
	counter++;
	this->lcdNumber->display(counter);
	this->lcdNumber->update();
}
