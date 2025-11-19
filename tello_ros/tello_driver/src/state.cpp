#include "tello_driver/state.h"

void parseStateString(const std::string stateString, std::map<std::string, double>& state) {
    int keyStart = 0;
    while (keyStart < (int) stateString.size()) {
        int keyValueCut = stateString.find(':', keyStart);
        int valueEnd = stateString.find(';', keyValueCut);

        std::string key = stateString.substr(keyStart, keyValueCut-keyStart);
        std::string valueString = stateString.substr(keyValueCut+1, valueEnd-keyValueCut-1);
        keyStart = valueEnd+1;

        // TODO: Check for exception
        double value = std::stod(valueString);

        state[key] = value;
    }
}
