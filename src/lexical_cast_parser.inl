#include <boost/lexical_cast.hpp>

template<class T>
LexicalCastParser<T>::~LexicalCastParser() {}

template<class T>
bool LexicalCastParser<T>::parse(const std::string& text, T& x) {
  try {
    x = boost::lexical_cast<T>(text);
  } catch (boost::bad_lexical_cast& ex) {
    return false;
  }

  return true;
}
