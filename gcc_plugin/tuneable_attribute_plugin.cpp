#include "gcc-plugin.h"
#include <stdlib.h>
#include "config.h"
#include "system.h"
#include "coretypes.h"
#include "tree.h"
#include "tree-pass.h"
#include "intl.h"
#include "toplev.h"
#include "plugin.h"
#include "diagnostic.h"
#include "stringpool.h"
#include "attribs.h"
#include <iostream>
#include <map>
#include "tree-pretty-print.h"
using namespace std;

int plugin_is_GPL_compatible;

// type var __attribute__((tuneable)) = default_value;
// type var __attribute__((tuneable, "short_name")) = default_value;
// type var __attribute__((param)) = default_value;
static struct attribute_spec tuneable_attr =
  { "tuneable", 0, 1, false,  false, false, false, NULL, NULL };

/* Plugin callback called during attribute registration */

static void register_attributes (void *event_data, void *data) 
{
  register_attribute (&tuneable_attr);
}

void parse_field_decl (tree field, std::string header) {
  std::string prefix = header;
  if (prefix != "") {
     prefix += ".";
  }
  if (TREE_CODE(TREE_TYPE(field)) == RECORD_TYPE) {
     for ( tree tf = TYPE_FIELDS(TREE_TYPE(field)); tf; tf = TREE_CHAIN(tf)) {
          if (TREE_CODE(tf) == FIELD_DECL) {
              //cerr << IDENTIFIER_POINTER(DECL_NAME(field));
              if (DECL_NAME(field)) {
                parse_field_decl(tf, prefix + IDENTIFIER_POINTER(DECL_NAME(field)));
              }
          }
      }
    return;
  }
//   cerr << get_tree_code_name(TREE_CODE(field)) << " " << 
//     get_tree_code_name(TREE_CODE(TREE_TYPE(field))) << " " <<
//     IDENTIFIER_POINTER(DECL_NAME(field)) << endl;

  for (tree attr = DECL_ATTRIBUTES (field); attr; attr = TREE_CHAIN (attr)) {
    tree attrname = TREE_PURPOSE (attr);
    tree attrargs = TREE_VALUE (attr);
    if(lookup_attribute("tuneable", DECL_ATTRIBUTES(field))) {
        tree type = TREE_TYPE(field);
        tree type_name_tree = TYPE_NAME(type);
        std::string type_name;
        std::string unsign = TYPE_UNSIGNED(type) ? "U" : "";        
        std::string size = std::to_string(TYPE_PRECISION(type));
        std::string const_str = TREE_READONLY(TREE_TYPE(field)) ? "const" : "";
        switch (TREE_CODE(type)) {
            case INTEGER_TYPE:
                type_name = unsign + "Int" + size;
                break;
            case REAL_TYPE:
                type_name = "Float";
                break;
            case BOOLEAN_TYPE:
                type_name = "Bool";
                break;
        }

      // Search param map for the variable and replace default value if found
      std::map<std::string, std::string> param_map;
      param_map["aaa"] = "22.3";
      std::string type_name2 = IDENTIFIER_POINTER(DECL_NAME(field));
      if (param_map.count(type_name2)) {
        cerr << "********************* kp" << std::endl; 
        print_generic_expr(stderr, DECL_INITIAL(field));
        cerr << std::endl;
        tree expr = DECL_INITIAL(field);
        cerr << get_tree_code_name(TREE_CODE(expr)) << std::endl;
        cerr <<  get_tree_code_name(TREE_CODE(TREE_OPERAND(expr, 0))) << std::endl;
        cerr <<  get_tree_code_name(TREE_CODE(TREE_OPERAND(TREE_OPERAND(expr, 0),0))) << std::endl;
        REAL_VALUE_TYPE &r = TREE_REAL_CST(TREE_OPERAND(TREE_OPERAND(expr, 0),0));
        cerr << r.uexp << std::endl;
        real_from_string(&r, param_map[type_name2].c_str());
        print_generic_expr(stderr, expr);
        cerr << std::endl;
        print_generic_expr(stderr, DECL_INITIAL(field));
        cerr << std::endl;

        // for (tree t = TREE_LIST(expr); t; t = TREE_CHAIN(t)) {
        //   cerr << t << " a " << get_tree_code_name(TREE_CODE(t)) << std::endl;
        // }
        cerr << "*********************" << std::endl;

      }


      cerr << "tuneable " << IDENTIFIER_POINTER(DECL_NAME(type_name_tree)) << (TREE_READONLY(TREE_TYPE(field)) ? " const" : "") << " " << prefix + IDENTIFIER_POINTER(DECL_NAME(field)) << std::endl;

      cout << const_str << " " << type_name << " " << prefix + IDENTIFIER_POINTER(DECL_NAME(field)) << std::endl;
    }
  }
}

void finish_type_callback (void*event_data, void*data)
{
  //cerr << "finish decl" << std::endl;
  tree decl = (tree)event_data;
  if (decl == NULL_TREE || decl == error_mark_node) {
		return;
  }
   //cerr << get_tree_code_name(TREE_CODE(decl)) << " " << IDENTIFIER_POINTER(DECL_NAME(decl)) << std::endl;

  tree type = TREE_TYPE(decl);

    if (TREE_CODE(decl) == VAR_DECL && TREE_CODE(type) == RECORD_TYPE) {
     // cerr << get_tree_code_name(TREE_CODE(type)) << " " << IDENTIFIER_POINTER(DECL_NAME(decl)) << endl;
            parse_field_decl(decl, "");
      
    }
}

int plugin_init (struct plugin_name_args *plugin_info,
             struct plugin_gcc_version *version)
{
  const char *plugin_name = plugin_info->base_name;
  register_callback(plugin_name,
        PLUGIN_FINISH_DECL, &finish_type_callback, 0);
  register_callback (plugin_name, PLUGIN_ATTRIBUTES, register_attributes, NULL);
  return 0;
}
