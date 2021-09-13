#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

string readFileIntoString(const string& path)
{
    auto ss = ostringstream{};
    ifstream input_file(path);
    if (!input_file.is_open())
    {
        cerr << "Could not open the file - '" << path << "'" << endl;
        exit(EXIT_FAILURE);
    }
    ss << input_file.rdbuf();
    return ss.str();
}


void read_record(const string& file_name)
{
    string filename(file_name);
    string file_contents;
    std::vector<std::vector<double>> control_inputs;

    file_contents = readFileIntoString(filename);

    istringstream sstream(file_contents);
    string record;

    while (std::getline(sstream, record))
    {
        istringstream line(record);
        std::vector<double> control_input;

        while (std::getline(line, record, ','))
        {
            control_input.push_back(stod(record));
        }

        control_inputs.push_back(control_input);
        control_input.clear();
    }

    for (const std::vector<double>& i : control_inputs)
    {
        for (double c : i)
        {
            cout<<c<<",";
        }
        cout<<endl;
    }
}

int main(int argc, char* argv[])
{
    string name = argv[1];
    read_record(name);
}
