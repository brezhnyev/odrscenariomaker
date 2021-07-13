#include "OSM2ODR.h"

#include <iostream>

using namespace std;
using namespace osm2odr;

int main(int argc, char ** argv)
{
    if (argc < 2)
    {
        cout << "Usage: " << argv[0] << " " << "path/to/file.osm" << endl;
    }
    cout << ConvertOSMToOpenDRIVE(argv[1]);

    return 0;
}