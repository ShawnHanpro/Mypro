#include <iostream>
#include <vector>
#include <limits>
#include <algorithm>

using namespace std;

class Solution {
public:
    bool canJump(vector<int>& nums) {
        int max_len = 0;
        for (int i = 0; i < nums.size(); ++i) {
            if (i < max_len) {
                max_len = std::max(i + nums[i], max_len);
                std::cout << max_len << " ";
                if (max_len >= nums.size() - 1) return true;
            }
        }
        return false;
    }
};


int main(int argc, char** argv) {
    
    std::cout << "hello vslam" << std::endl;
    vector<int> nums={2,3,1,1,4};
    int val = 2;
    Solution sol;
    bool res = sol.canJump(nums);
    // for (auto& n : nums) {
    //     std::cout << n << " ";
    // }
    std::cout << std::boolalpha << res << std::endl;

    return 0;
}