#include <iostream>
#include <stdio.h>
#include <string.h>

#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

string mystr;
int main(){
	cout << "hello"<<endl;
	getline(cin,mystr);
	cout << mystr << endl;
	return 0;
}
