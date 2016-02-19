#ifndef OBJ_POOL_HPP
#define OBJ_POOL_HPP

#include <vector>
#include "IntWrapper.hpp"

template <typename InfoType>
class ObjPool {
public:
	struct tag;
	using size_type = unsigned;
	using uid_type = IntWrapper<tag, size_type>;
	
	uid_type add(InfoType newInfo) {
		if (removedUids_.size()) {
			uid_type reuse = removedUids_.back();
			removedUids_.pop_back();
			infos_[reuse] = newInfo;
			alive_[reuse] = true;
			++size_;
			return reuse;
		}
		else {
			infos_.push_back(newInfo);
			alive_.push_back(true);
			++size_;
			return uid_type(size_-1);
		}
	}

	void remove(uid_type uid) {
		removedUids_.push_back(uid);
		alive_[uid] = false;
	}

	InfoType info(uid_type uid) {
		return infos_[uid];
	}
	
private:
	size_type size_;
	std::vector<InfoType> infos_;
	std::vector<uid_type> removedUids_;
	std::vector<bool> alive_;
};


#endif // OBJ_POOL_HPP
