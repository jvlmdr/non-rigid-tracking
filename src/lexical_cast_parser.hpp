#ifndef LEXICAL_CAST_PARSER_HPP_
#define LEXICAL_CAST_PARSER_HPP_

#include "parser.hpp"

template<class T>
class LexicalCastParser : public Parser<T> {
  public:
    ~LexicalCastParser();
    bool parse(const std::string& text, T& x);
};

#include "lexical_cast_parser.inl"

#endif
