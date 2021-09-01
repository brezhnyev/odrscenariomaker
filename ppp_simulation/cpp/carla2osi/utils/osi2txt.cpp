#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

int main(int argc, char ** argv)
{
    if (argc < 2)
    {
        cout << "Syntax: " << argv[0] << " path/to/file.osi" << endl;
        return 0;
    }

    ifstream ifs(argv[1], ios::binary);
    ofstream ofs((string(argv[1]).substr(0, string(argv[1]).size()-3) + "txt").c_str(), ios::binary);

    stringstream ss;
    string delim = "$$__$$";

    union int2bytes
    {
        int i;
        char b[4];
    } i2b;

    vector<char> message;

    while (ifs.read(i2b.b, 4))
    {
        message.resize(i2b.i);
        ifs.read(&message[0], i2b.i);
        ofs.write(&message[0], i2b.i);
        ofs << delim;
    }

    ifs.close();
    ofs.close();

    return 0;

}