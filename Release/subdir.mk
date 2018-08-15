################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../PointPolygonInteresector.cpp \
../RedLegoDetector.cpp \
../handGesture.cpp \
../main.cpp \
../main2.cpp \
../myImage.cpp \
../roi.cpp 

OBJS += \
./PointPolygonInteresector.o \
./RedLegoDetector.o \
./handGesture.o \
./main.o \
./main2.o \
./myImage.o \
./roi.o 

CPP_DEPS += \
./PointPolygonInteresector.d \
./RedLegoDetector.d \
./handGesture.d \
./main.d \
./main2.d \
./myImage.d \
./roi.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/Cellar/opencv/3.4.1_5/include -O3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


