/*
 * RangeFinder_SR04.h
 *
 *  Created on: Jul 13, 2016
 *      Author: chris
 */

#ifndef RANGEFINDER_SR04_H_
#define RANGEFINDER_SR04_H_

class RangeFinder_SR04: public RangeFinder {
public:
	bool init();
	bool update();
	bool update(uint16_t &distance);
};


#endif /* RANGEFINDER_SR04_H_ */
