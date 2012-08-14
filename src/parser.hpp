#ifndef PARSER_HPP_
#define PARSER_HPP_

template<class T>
class Parser {
  public:
    virtual ~Parser() {}
    virtual bool parse(const std::string& text, T& x) = 0;
};

#endif
