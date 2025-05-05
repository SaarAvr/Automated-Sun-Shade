/*
 * cppTest.cpp
 *
 *  Created on: Feb 3, 2025
 *      Author: saar
 */
#include "cppTest.h"
#include "iostream"

class MyClass{
public:
	int value = 1;
};

int func(){
	MyClass obj;
	obj.value = 5;
	std::cout << "value is: " << obj.value << std::endl;
	return obj.value;
}


