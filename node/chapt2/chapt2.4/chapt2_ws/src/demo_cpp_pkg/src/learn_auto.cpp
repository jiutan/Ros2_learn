#include<iostream>

int main(){

    auto x = 5 ;            // 此时，auto 为 int
    auto y = 3.14f;         // 此时，auto 为 double
    auto z = 'a';           // 此时，auto 为 char

    std::cout << x << y << z << std::endl;
}