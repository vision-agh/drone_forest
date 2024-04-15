#include <drone_forest/instructions.h>
#include <math.h>

void fillInstructionSet(Instructions::Set& set)
{
  auto minus = [](double a, double b) -> double { return a - b; };
  auto add = [](double a, double b) -> double { return a + b; };
  auto mult = [](double a, double b) -> double { return a * b; };
  auto div = [](double a, double b) -> double { return a / b; };
  auto max = [](double a, double b) -> double { return std::max(a, b); };
  auto ln = [](double a) -> double { return std::log(a); };
  auto exp = [](double a) -> double { return std::exp(a); };
  auto cos = [](double a) -> double { return std::cos(a); };
  auto sin = [](double a) -> double { return std::sin(a); };
  auto tan = [](double a) -> double { return std::tan(a); };
  //   auto pi = [](double a) -> double { return M_PI; };
  //   auto multByConst = [](double a, Data::Constant c) -> double
  //   { return a * (double)c / 10.0; };
  auto relu = [](double a) -> double { return a > 0 ? a : 0; };

  set.add(*(new Instructions::LambdaInstruction<double, double>(
      minus, "$0 = $1 - $2;")));
  set.add(*(new Instructions::LambdaInstruction<double, double>(
      add, "$0 = $1 + $2;")));
  set.add(*(new Instructions::LambdaInstruction<double, double>(
      mult, "$0 = $1 * $2;")));
  set.add(*(new Instructions::LambdaInstruction<double, double>(
      div, "$0 = $1 / $2;")));
  set.add(*(new Instructions::LambdaInstruction<double, double>(
      max, "$0 = (($1) < ($2)) ? ($2) : ($1);")));
  set.add(*(new Instructions::LambdaInstruction<double>(exp, "$0 = exp($1);")));
  set.add(*(new Instructions::LambdaInstruction<double>(ln, "$0 = log($1);")));
  set.add(*(new Instructions::LambdaInstruction<double>(cos, "$0 = cos($1);")));
  set.add(*(new Instructions::LambdaInstruction<double>(sin, "$0 = sin($1);")));
  set.add(*(new Instructions::LambdaInstruction<double>(tan, "$0 = tan($1);")));
  //   set.add(*(new Instructions::LambdaInstruction<double, Data::Constant>(
  //       multByConst, "$0 = $1 * ((double)($2) / 10.0);")));
  //   set.add(*(new Instructions::LambdaInstruction<double>(pi, "$0 =
  //   M_PI;")));
  set.add(*(new Instructions::LambdaInstruction<double>(
      relu, "$0 = ($1 > 0) ? $1 : 0;")));
}