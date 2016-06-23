/*!
*	\file	Diagnostic.h
*	\date	2016.01.20
*	\author	Keunjun Choi(ckj.monikaru@gmail.com)
*	\brief	Define useful macros and functions to diagnose a situation during execution.
*/

#pragma once

#ifndef NLOG

#include <iostream>
#include <cassert>
#include <ctime>
#include <omp.h>

#define LOGIF(statement,message)\
	do{\
		if(!(statement)){std::cout<<(message)<<std::endl;}\
		assert((statement)&&(message));\
	}while(false);

#define LOG(message)\
	std::cout<<(message)<<std::endl;

#define LOG_VALUE(message1,message2)\
	std::cout<<(message1)<<(message2)<<std::endl;

#else

#define LOGIF(statement,message) ;
#define LOG(message) ;

#endif

#define PERFORM_TEST(loop,iteration)\
	do{\
		std::cout<<"Performance Test Start"<<std::endl;\
		double begin  = omp_get_wtime();\
		for(int i = 0 ; i < iteration ; i++){loop}\
		double end = omp_get_wtime();\
		std::cout<<"Performance Test End"<<std::endl;\
		std::cout<<"Computation Time:"<<(end-begin)*1000<<"\tms"<<std::endl;\
	}while(false);