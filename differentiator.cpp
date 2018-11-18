#include <iostream>
#include <cassert>
#include <string>
#include <limits>
#include <memory>
#include <cmath>

namespace Mathematics {

// ****************
// MAGIC CONSTANTS & FUNCTIONS
// ****************

constexpr double EPS = 1e-6;

bool are_equal(double x, double y) {
  return fabs(x - y) < EPS;
}

// ****************
//  NODE DESCRIPTION
// ****************

enum ENodeType {
  T_CONSTANT,
  T_VARIABLE,
  T_OPERATION,
};

enum EOperationType {
  OP_ADD,
  OP_SUBTRACT,
  OP_MULTIPLY,
  OP_DIVIDE,
  OP_SIN,
  OP_COS,
  OP_LOG,
  OP_NONE
};

std::string const op_names[] = {"+", "-", "*", "/", "sin", "cos", "log"};
short const op_arity[] = {2, 2, 2, 2, 1, 1, 1};

struct Expression {

  std::unique_ptr<Expression> left_;
  std::unique_ptr<Expression> right_;

  ENodeType node_type_;

  EOperationType operation_type_;
  double constant_;
  char variable_;

  Expression() = delete;

  explicit Expression(EOperationType operation_type,
                      std::unique_ptr<Expression> left,
                      std::unique_ptr<Expression> right = nullptr) :
      left_(std::move(left)),
      right_(std::move(right)),
      node_type_(T_OPERATION),
      operation_type_(operation_type),
      constant_(std::numeric_limits<double>::infinity()),
      variable_(0)
  {} // TODO : сделать ассерт если унарный оператор то right = nullptr

  explicit Expression(double constant) :
      left_(nullptr),
      right_(nullptr),
      node_type_(T_CONSTANT),
      operation_type_(OP_NONE),
      constant_(constant),
      variable_(0)
  {}

  explicit Expression(char variable) :
      left_(nullptr),
      right_(nullptr),
      node_type_(T_VARIABLE),
      operation_type_(OP_NONE),
      constant_(std::numeric_limits<double>::infinity()),
      variable_(variable)
  {}

  Expression(Expression&& node) noexcept :
      node_type_(node.node_type_),
      operation_type_(node.operation_type_),
      constant_(node.constant_),
      variable_(node.variable_)
  {
    auto left_new = std::move(node.left_);
    auto right_new = std::move(node.right_);

    left_  = std::move(left_new);
    right_ = std::move(right_new);
  }

  Expression(const Expression& node) noexcept :
      left_(nullptr),
      right_(nullptr),
      node_type_(node.node_type_),
      operation_type_(node.operation_type_),
      constant_(node.constant_),
      variable_(node.variable_)
  {}

  Expression& operator =(Expression&& node) noexcept {
    auto left_new = std::move(node.left_);
    auto right_new = std::move(node.right_);

    node_type_ = node.node_type_;
    operation_type_ = node.operation_type_;
    constant_ = node.constant_;
    variable_ = node.variable_;

    left_  = std::move(left_new);
    right_ = std::move(right_new);
    return *this;
  }

  Expression& operator =(const Expression& node) noexcept {
    left_.reset();
    right_.reset();
    node_type_ = node.node_type_;
    operation_type_ = node.operation_type_;
    constant_ = node.constant_;
    variable_ = node.variable_;
    return *this;
  }

  std::unique_ptr<Expression> deep_copy() const {
    return deep_copy(*this);
  }

  static std::unique_ptr<Expression> deep_copy(const Expression& node) {
    auto node_copied = std::make_unique<Expression>(node);
    if (node.left_) {
      node_copied->left_  = std::move(deep_copy(*node.left_));
    }
    if (node.right_) {
      node_copied->right_ = std::move(deep_copy(*node.right_));
    }
    return node_copied;
  }

  auto to_constant(double constant) {
    node_type_      = T_CONSTANT;
    operation_type_ = OP_NONE;
    constant_       = constant;
    return std::make_pair(std::move(left_), std::move(right_));
  }
};

#define L(node) ((node).left_)
#define R(node) ((node).right_)
#define has_type(node, type) ((node).node_type_ == (type))
#define has_op(node, op) ((node).operation_type_ == (op))
#define has_arity(node, arity) (op_arity[(node).operation_type_] == (arity))

// **************************************
// CUSTOM OPERATORS FOR EXPRESSIONS
// **************************************

std::unique_ptr<Expression> operator +(std::unique_ptr<Expression> lhs, std::unique_ptr<Expression> rhs) {
  return std::make_unique<Expression>(OP_ADD, std::move(lhs), std::move(rhs));
}

std::unique_ptr<Expression> operator -(std::unique_ptr<Expression> lhs, std::unique_ptr<Expression> rhs) {
  return std::make_unique<Expression>(OP_SUBTRACT, std::move(lhs), std::move(rhs));
}

std::unique_ptr<Expression> operator *(std::unique_ptr<Expression> lhs, std::unique_ptr<Expression> rhs) {
  return std::make_unique<Expression>(OP_MULTIPLY, std::move(lhs), std::move(rhs));
}

std::unique_ptr<Expression> operator /(std::unique_ptr<Expression> lhs, std::unique_ptr<Expression> rhs) {
  return std::make_unique<Expression>(OP_DIVIDE, std::move(lhs), std::move(rhs));
}

std::unique_ptr<Expression> sin(std::unique_ptr<Expression> lhs) {
  return std::make_unique<Expression>(OP_SIN, std::move(lhs));
}

std::unique_ptr<Expression> cos(std::unique_ptr<Expression> lhs) {
  return std::make_unique<Expression>(OP_COS, std::move(lhs));
}

std::unique_ptr<Expression> log(std::unique_ptr<Expression> lhs) {
  return std::make_unique<Expression>(OP_LOG, std::move(lhs));
}

// *****************
// DIFFERENTIATING
// *****************

#define deriveL derive(*node.left_, w_respect_to)
#define deriveR derive(*node.right_, w_respect_to)
#define copyL node.left_->deep_copy()
#define copyR node.right_->deep_copy()

std::unique_ptr<Expression> derive(const Expression&, char);

std::unique_ptr<Expression> derive_constant(const Expression& node, char w_respect_to) {
  assert(has_type(node, T_CONSTANT));

  return std::make_unique<Expression>(0.);
}

std::unique_ptr<Expression> derive_variable(const Expression& node, char w_respect_to) {
  assert(has_type(node, T_VARIABLE));

  double val = std::numeric_limits<double>::infinity();
  if (node.variable_ == w_respect_to) {
    val = 1.;
  } else {
    val = 0.;
  }

  return std::make_unique<Expression>(val);
}

std::unique_ptr<Expression> derive_op_add(const Expression& node, char w_respect_to) {
  assert(has_type(node, T_OPERATION));
  assert(has_op(node, OP_ADD));

  return deriveL + deriveR;
}

std::unique_ptr<Expression> derive_op_subtract(const Expression& node, char w_respect_to) {
  assert(has_type(node, T_OPERATION));
  assert(has_op(node, OP_SUBTRACT));

  return deriveL - deriveR;
}

std::unique_ptr<Expression> derive_op_multiply(const Expression& node, char w_respect_to) {
  assert(has_type(node, T_OPERATION));
  assert(has_op(node, OP_MULTIPLY));

  return deriveL * copyR + copyL * deriveR;
}

std::unique_ptr<Expression> derive_op_divide(const Expression& node, char w_respect_to) {
  assert(has_type(node, T_OPERATION));
  assert(has_op(node, OP_DIVIDE));

  return (deriveL * copyR - deriveR * copyL) / (copyR * copyR);
}

std::unique_ptr<Expression> derive_op_sin(const Expression& node, char w_respect_to) {
  assert(has_type(node, T_OPERATION));
  assert(has_op(node, OP_SIN));

  return deriveL * cos(copyL);
}

std::unique_ptr<Expression> derive_op_cos(const Expression& node, char w_respect_to) {
  assert(has_type(node, T_OPERATION));
  assert(has_op(node, OP_COS));

  return std::make_unique<Expression>(-1.) * deriveL * sin(copyL);
}

std::unique_ptr<Expression> derive_op_log(const Expression& node, char w_respect_to) {
  assert(has_type(node, T_OPERATION));
  assert(has_op(node, OP_LOG));

  return deriveL / copyL;
}

std::unique_ptr<Expression> derive_operation(const Expression& node, char w_respect_to) {
  assert(has_type(node, T_OPERATION));

  switch (node.operation_type_) {
    case OP_ADD:
      return derive_op_add(node, w_respect_to);
    case OP_SUBTRACT:
      return derive_op_subtract(node, w_respect_to);
    case OP_MULTIPLY:
      return derive_op_multiply(node, w_respect_to);
    case OP_DIVIDE:
      return derive_op_divide(node, w_respect_to);
    case OP_SIN:
      return derive_op_sin(node, w_respect_to);
    case OP_COS:
      return derive_op_cos(node, w_respect_to);
    case OP_LOG:
      return derive_op_log(node, w_respect_to);
    default:
      assert(false);
  }
}

std::unique_ptr<Expression> derive(const Expression& node, char w_respect_to) {
  switch (node.node_type_) {
    case T_CONSTANT:
      return derive_constant(node, w_respect_to);
    case T_VARIABLE:
      return derive_variable(node, w_respect_to);
    case T_OPERATION:
      return derive_operation(node, w_respect_to);
  }
}

// **********************
// SIMPLIFYING EXPRESSIONS
// **********************

double calculate_unary_operation(EOperationType operation_type, double arg) {
  assert(op_arity[operation_type] == 1);

  switch (operation_type) {
    case OP_SIN:
      return std::sin(arg);
    case OP_COS:
      return std::cos(arg);
    case OP_LOG:
      return std::log(arg);
    default:
      assert(false);
  }
}

double calculate_binary_operation(EOperationType operation_type, double arg1, double arg2) {
  assert(op_arity[operation_type] == 2);

  switch (operation_type) {
    case OP_ADD:
      return arg1 + arg2;
    case OP_SUBTRACT:
      return arg1 - arg2;
    case OP_MULTIPLY:
      return arg1 * arg2;
    case OP_DIVIDE:
      return arg1 / arg2;
    default:
      assert(false);
  }
}

bool simplify_operations_with_constants(Expression& node) {
  bool flag = false;

  if (node.left_)  flag |= simplify_operations_with_constants(*node.left_);
  if (node.right_) flag |= simplify_operations_with_constants(*node.right_);

  if (!has_type(node, T_OPERATION)) return flag;

  if (!has_type(*node.left_, T_CONSTANT)) return flag;

  if (has_arity(node, 1)) {

    double arg = node.left_->constant_;
    double constant = calculate_unary_operation(node.operation_type_, arg);
    node.to_constant(constant);

    flag = true;
  } else if (has_arity(node, 2)) {

    if (!has_type(*node.right_, T_CONSTANT))
      return flag;

    double arg1 = node.left_->constant_;
    double arg2 = node.right_->constant_;
    double constant = calculate_binary_operation(node.operation_type_, arg1, arg2);
    node.to_constant(constant);

    flag = true;
  }

  return flag;
}

bool simplify_add_zero(Expression& node) {
  bool flag = false;

  if (node.left_)  flag |= simplify_add_zero(*node.left_);
  if (node.right_) flag |= simplify_add_zero(*node.right_);

  if (!has_op(node, OP_ADD)) {
    return flag;
  }

  if (has_type(*node.left_, T_CONSTANT) && are_equal(node.left_->constant_, 0)) {
    node = std::move(*node.right_);
    flag = true;
  } else if (has_type(*node.right_, T_CONSTANT) && are_equal(node.right_->constant_, 0)) {
    node = std::move(*(node.left_));
    flag = true;
  }

  return flag;
}

bool simplify_multiply_divide_one(Expression& node) {
  bool flag = false;

  if (node.left_)  flag |= simplify_multiply_divide_one(*node.left_);
  if (node.right_) flag |= simplify_multiply_divide_one(*node.right_);

  if (!has_op(node, OP_MULTIPLY) && !has_op(node, OP_DIVIDE)) {
    return flag;
  }

  if (has_type(*node.left_, T_CONSTANT) && are_equal(node.left_->constant_, 1)) {
    node = std::move(*node.right_);
    flag = true;
  } else if (has_type(*node.right_, T_CONSTANT) && are_equal(node.right_->constant_, 1)) {
    node = std::move(*node.left_);
    flag = true;
  }

  return flag;
}

bool simplify_multiply_zero(Expression& node) {
  bool flag = false;

  if (node.left_)  flag |= simplify_multiply_zero(*node.left_);
  if (node.right_) flag |= simplify_multiply_zero(*node.right_);

  if (!has_op(node, OP_MULTIPLY)) {
    return flag;
  }

  if (has_type(*node.left_, T_CONSTANT) && are_equal(node.left_->constant_, 0)) {
    node = std::move(Expression(0.));
    flag = true;
  } else if (has_type(*node.right_, T_CONSTANT) && are_equal(node.right_->constant_, 0)) {
    node = std::move(Expression(0.));
    flag = true;
  }

  return flag;
}

bool simplify(Expression& node) {
  bool current_flag = false;
  bool flag = false;
  do {
    current_flag = false;
    current_flag |= simplify_operations_with_constants(node);
    current_flag |= simplify_add_zero(node);
    current_flag |= simplify_multiply_divide_one(node);
    current_flag |= simplify_multiply_zero(node);
    flag |= current_flag;
  } while(current_flag);
  return flag;
}

// *****************
// SERIALIZATION
// *****************

namespace ExpressionSerDe {

template <class OutputIt>
OutputIt serialize(const Expression&, OutputIt);

template <class OutputIt>
OutputIt serialize_constant(const Expression& node, OutputIt iter) {
  assert(has_type(node, T_CONSTANT));

  auto constant_str = std::to_string(node.constant_);
  return std::copy(constant_str.begin(), constant_str.end(), iter);
}

template <class OutputIt>
OutputIt serialize_variable(const Expression& node, OutputIt iter) {
  assert(has_type(node, T_VARIABLE));

  *iter++ = node.variable_;
  return iter;
}

template <class OutputIt>
OutputIt serialize_operation_name(const Expression& node, OutputIt iter) {
  assert(has_type(node, T_OPERATION));

  const auto& name = op_names[node.operation_type_];
  return std::copy(name.begin(), name.end(), iter);
}

template <class OutputIt>
OutputIt serialize_unary_operation(const Expression& node, OutputIt iter) {
  assert(has_type(node, T_OPERATION));
  assert(has_arity(node, 1));

  iter = serialize_operation_name(node, iter);
  *iter++ = '(';
  iter = serialize(*L(node), iter);
  *iter++ = ')';

  return iter;
}

template <class OutputIt>
OutputIt serialize_binary_operation(const Expression& node, OutputIt iter) {
  assert(has_type(node, T_OPERATION));
  assert(has_arity(node, 2));

  *iter++ = '(';
  iter = serialize(*L(node), iter);
  *iter++ = ')';

  iter = serialize_operation_name(node, iter);

  *iter++ = '(';
  iter = serialize(*R(node), iter);
  *iter++ = ')';

  return iter;
}

template <class OutputIt>
OutputIt serialize(const Expression& node, OutputIt iter) {
  switch (node.node_type_) {
    case T_CONSTANT:
      return serialize_constant(node, iter);
    case T_VARIABLE:
      return serialize_variable(node, iter);
    case T_OPERATION: {
      auto operation_arity = op_arity[node.operation_type_];
      if (operation_arity == 1) {
        return serialize_unary_operation(node, iter);
      } else if (operation_arity == 2) {
        return serialize_binary_operation(node, iter);
      } else {
        assert(false);
      }
    }
    default:
      assert(false);
  }
}

std::string serialize_string(const Expression& node) {
  std::string str;
  serialize(node, std::back_inserter(str));
  return str;
}

// *****************
// DESERIALIZATION
// *****************

template <class InputIt>
std::unique_ptr<Expression> deserialize(InputIt, InputIt);

template <class InputIt>
double deserialize_double(InputIt begin, InputIt end) {
  std::string str_double(begin, end);
  try {
    size_t stod_length = 0;
    double deserialized = std::stod(str_double, &stod_length);
    if (stod_length != str_double.length()) {
      throw std::invalid_argument("");
    }
    return deserialized;
  } catch (const std::invalid_argument&) {
    throw std::invalid_argument("Failed to convert '" + str_double + "' into double"); // TODO: custom exception
  } catch (const std::out_of_range&) {
    throw std::out_of_range("Number '" + str_double + "' is too big"); // TODO: custom exception
  }
}

template <class InputIt>
std::unique_ptr<Expression> deserialize_binary_operation(InputIt begin, InputIt end) {
  InputIt it = begin;
  assert(*it == '(');

  ++it;
  int balance = 1;

  for (; balance != 0 && it != end; ++it) {
    if (*it == '(') ++balance;
    if (*it == ')') --balance;
  }

  if (it == end) {
    throw std::invalid_argument("Unexpected end while parsing binary operation");
  }

  InputIt operation_sign_pos = it;

  EOperationType operation_type = OP_NONE;
  switch (*it) {
    case '+':
      operation_type = OP_ADD;
      break;
    case '-':
      operation_type = OP_SUBTRACT;
      break;
    case '*':
      operation_type = OP_MULTIPLY;
      break;
    case '/':
      operation_type = OP_DIVIDE;
      break;
    default:
      throw std::invalid_argument("'" + std::string(1, *it) + "' expected to be one of binary operations: +, -, *, /");
  }

  ++it;
  if (it == end || *it != '(') {
    throw std::invalid_argument("Failed to find argument after binary operation '" + op_names[operation_type] + "'");
  }

  balance = 1;
  ++it;

  for (; balance != 0 && it != end; ++it) {
    if (*it == '(') ++balance;
    if (*it == ')') --balance;
  }

  if (balance != 0) {
    throw std::invalid_argument("Unexpected end while parsing binary operation");
  }

  if (it != end) {
    throw std::invalid_argument("Found useless characters after second argument");
  }

  auto arg1 = deserialize(begin + 1, operation_sign_pos - 1);
  auto arg2 = deserialize(operation_sign_pos + 2, end - 1);
  if (!arg1 || !arg2) {
    throw std::invalid_argument("Missing argument for operator " + op_names[operation_type]);
  }
  return std::make_unique<Expression>(operation_type, std::move(arg1), std::move(arg2));
}

template <class InputIt>
std::unique_ptr<Expression> deserialize(const InputIt begin, const InputIt end) {
  if (end == begin) {
    return nullptr;
  }

  // CHECK FOR NUMBER

  char ch = *begin;
  if (ch == '-' || ch == '+' || std::isdigit(ch)) {
    return std::make_unique<Expression>(deserialize_double(begin, end));
  }

  // CHECK FOR BINARY OPERATION

  if (ch == '(') {
    return deserialize_binary_operation(begin, end);
  }

  // CHECK FOR UNARY OPERATION

  EOperationType operation_type = OP_NONE;

  if (std::distance(begin, end) >= 5) {
    std::string prefix(begin, begin + 3);
    if (prefix == "sin") {
      operation_type = OP_SIN;
    } else if (prefix == "cos") {
      operation_type = OP_COS;
    } else if (prefix == "log") {
      operation_type = OP_LOG;
    }
  }

  if (operation_type != OP_NONE) {
    auto arg = deserialize(begin + 4, end - 1);
    if (!arg) {
      throw std::invalid_argument("Argument for " + op_names[operation_type] + " is missing");
    }
    return std::make_unique<Expression>(operation_type, std::move(arg));
  }

  // CHECK FOR VARIABLE
  if (std::distance(begin, end) == 1 && std::isalpha(*begin)) {
    return std::make_unique<Expression>(*begin);
  }

  throw std::invalid_argument("Failed to parse following: " + std::string(begin, end));


}


}

}

int main() {

  /*auto v2 = std::make_unique<Mathematics::Expression>(1.);
  auto v3 = std::make_unique<Mathematics::Expression>('x');
  auto v1 = std::make_unique<Mathematics::Expression>(Mathematics::OP_ADD, std::move(v2), std::move(v3));

  auto v1_ = v1->deep_copy();

  std::cout << Mathematics::ExpressionSerDe::serialize_string(*v1) << '\n';
  std::cout << Mathematics::ExpressionSerDe::serialize_string(*v1_) << '\n';

  auto s = std::move(v1) * std::move(v1_);
  std::cout << Mathematics::ExpressionSerDe::serialize_string(*s) << '\n';

  auto sin = Mathematics::sin(std::move(s));
  std::cout << Mathematics::ExpressionSerDe::serialize_string(*sin) << '\n';

  auto der = Mathematics::derive(*sin, 'x');
  std::cout << Mathematics::ExpressionSerDe::serialize_string(*der) << '\n';

  std::cout << Mathematics::simplify(*der) << '\n';
  std::cout << Mathematics::ExpressionSerDe::serialize_string(*der) << '\n';*/

  std::string str = "sin(((7)*(x))+(cos((5)*(5))))";

  auto v = Mathematics::ExpressionSerDe::deserialize(str.begin(), str.end());
  if (v != nullptr) {
    std::cout << Mathematics::ExpressionSerDe::serialize_string(*v) << '\n';
  }


  Mathematics::simplify(*v);
  std::cout << Mathematics::ExpressionSerDe::serialize_string(*v) << '\n';

  auto der = Mathematics::derive(*v, 'x');
  Mathematics::simplify(*der);
  std::cout << Mathematics::ExpressionSerDe::serialize_string(*der) << '\n';

  return 0;
}
