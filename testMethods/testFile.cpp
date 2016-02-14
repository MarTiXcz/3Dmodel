#include <random>
#include <time.h>
#include <thread>
#include <chrono>
#include <boost/random.hpp>

#include "stdafx.h"

double random01(void) {
	boost::random::mt19937 rng;
	rng.seed(time(nullptr));
	static boost::uniform_01<boost::mt19937> rn(rng);
	return rn();
}

int main(void) {
	srand(time(nullptr));
	while (1) {
		printf("%d\n", rand() % 255);
		printf("boost:%f\n", random01());
		printf("boost:%f\n", random01());
		printf("boost:%f\n", random01());

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	;
}