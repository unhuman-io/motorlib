#ifndef UNHUMAN_MOTORLIB_CSTACK_H_
#define UNHUMAN_MOTORLIB_CSTACK_H_

#include <atomic>

// A circular stack. If data is written by one thread and read by one other thread then data is read from the top without worrying about thread safety.
template <class T, int size=100>
class CStack {
 public:
	CStack() = default;
	void copy(const CStack &stack) {
		pos_ = stack.pos_.load(std::memory_order_acquire);
		for(int i=0;i<size;i++) {
			data_[pos_] = stack.data_[pos_];
			pos_++;
			if (pos_ >= size) {
				pos_ = 0;
			}
		}
		
		future_pos_ = stack.future_pos_;
	}
    void push(T const &t) {
		next() = t;
		finish();	
	}
	T &next() {
		future_pos_ = pos_.load(std::memory_order_acquire) + 1;
		if (future_pos_ >= size) {
			future_pos_ = 0;
		}
		return data_[future_pos_];
	}
	void finish() {
		pos_.store(future_pos_, std::memory_order_release);
	}
	const T &top() const { // return a copy of the data
		return data_[pos_.load(std::memory_order_acquire)];
	}
 private:
	T data_[size] = {};
	std::atomic<int> pos_ = {0};
	int future_pos_ = {0};
};

#endif  // UNHUMAN_MOTORLIB_CSTACK_H_
