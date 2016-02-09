#include <random>
#include <time.h>
#include <thread>
#include <chrono>

int main(void) {
	srand(time(NULL));
	while (1) {
		printf("%d\n", rand() % 255);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	;
}