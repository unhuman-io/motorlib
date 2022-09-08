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
#include "gimple.h"
#include "gimple-iterator.h"
#include "basic-block.h"
using namespace std;

int plugin_is_GPL_compatible;

std::map<std::string, tree> param_map;

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

void parse_param_decl (tree field, std::string header = "") {
    std::string prefix = header;
  if (prefix != "") {
     prefix += ".";
  }
    if (TREE_CODE(TREE_TYPE(field)) == RECORD_TYPE) {
     for ( tree tf = TYPE_FIELDS(TREE_TYPE(field)); tf; tf = TREE_CHAIN(tf)) {
          if (TREE_CODE(tf) == FIELD_DECL) {
              //cerr << IDENTIFIER_POINTER(DECL_NAME(field));
              if (DECL_NAME(field)) {

                std::string var_name = IDENTIFIER_POINTER(DECL_NAME(field));
                if (var_name == "param1") {
                  var_name = "";
                  cerr << "del" << var_name << std::endl;
                }
                cerr << var_name << std::endl;
                parse_param_decl(tf, prefix + var_name);
              }
          }
      }
    return;
  }

  std::string var_name = IDENTIFIER_POINTER(DECL_NAME(field));
  param_map[prefix + var_name] = DECL_INITIAL(field);
  cerr << "added: " << prefix + var_name << std::endl;
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
                cerr << "tf: " << DECL_INITIAL(tf) << endl;
                std::string var_name = IDENTIFIER_POINTER(DECL_NAME(field));
                parse_field_decl(tf, prefix + var_name);
                cerr << "tf2: " << DECL_INITIAL(tf) << endl;
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
      std::string var_name = IDENTIFIER_POINTER(DECL_NAME(field));
      std::string type_name2 = prefix + var_name;
      if (param_map.count(type_name2)) {
        cerr << "*********************" << std::endl; 
        print_generic_expr(stderr, DECL_INITIAL(field));
        cerr << std::endl;
        //tree field2 = copy_node(field);
        //field = field2;
        tree node = make_node(NON_LVALUE_EXPR);
        node = copy_node(param_map[type_name2]);
        DECL_INITIAL(field) = node; //param_map[type_name2];
        // tree expr = copy_node(DECL_INITIAL(field));
        // cerr << get_tree_code_name(TREE_CODE(expr)) << std::endl;
        // cerr <<  get_tree_code_name(TREE_CODE(TREE_OPERAND(expr, 0))) << std::endl;
        // cerr <<  get_tree_code_name(TREE_CODE(TREE_OPERAND(TREE_OPERAND(expr, 0),0))) << std::endl;
        // REAL_VALUE_TYPE &r = TREE_REAL_CST(TREE_OPERAND(TREE_OPERAND(expr, 0),0));
        // cerr << "real address: " << &r << std::endl;
        // cerr << r.uexp << std::endl;
        // real_from_string(&r, param_map[type_name2].c_str());
        // //expr_from_string(expr, "1+2");
        // print_generic_expr(stderr, expr);
        // cerr << std::endl;
        print_generic_expr(stderr, DECL_INITIAL(field));
        cerr << std::endl;

        // for (tree t = TREE_LIST(expr); t; t = TREE_CHAIN(t)) {
        //   cerr << t << " a " << get_tree_code_name(TREE_CODE(t)) << std::endl;
        // }
        cerr << "*********************" << std::endl;

      }
      if (var_name == "kp_") {
        print_generic_expr(stderr, DECL_INITIAL(field));
        cerr << std::endl;
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
  if (TREE_CODE(decl) == VAR_DECL) {
    std::string name = IDENTIFIER_POINTER(DECL_NAME(decl));
    cerr << "var: " << name << " " << get_tree_code_name(TREE_CODE(type)) << std::endl;

  }

    if (TREE_CODE(decl) == VAR_DECL && TREE_CODE(type) == RECORD_TYPE) {
     // cerr << get_tree_code_name(TREE_CODE(type)) << " " << IDENTIFIER_POINTER(DECL_NAME(decl)) << endl;
      
      std::string name = IDENTIFIER_POINTER(DECL_NAME(decl));
      if (name == "param1") {
        cerr << "============= param1" << std::endl;
        parse_param_decl(decl);
      } else {
            parse_field_decl(decl, "");
      }
      
    }
}

 static unsigned myplugin_exec(void)
    {
       unsigned i;
       const_tree str, op;
       basic_block bb;
       gimple stmt;
       gimple_stmt_iterator gsi;

       cerr << "PASS !!!!!!!!!!!!!!!!!!!!!!!" << endl;

      //  FOR_EACH_BB(bb)
      //    for (gsi=gsi_start_bb(bb); !gsi_end_p(gsi); gsi_next(&gsi))
      //    {
      //        stmt = gsi_stmt(gsi);
      //        for (i=0; i<gimple_num_ops(stmt); ++i)
      //          if ((op = gimple_op(stmt, i)) && (str = 0)) {//is_str_cst(op)))
      //            //spell_check(stmt, str);
      //          }
      //    }

       return 0;
    }

    /* See tree-pass.h for a list and descriptions for the fields of this struct */
    static struct gimple_opt_pass myplugin_pass = 
    {
        .pass{.type = GIMPLE_PASS,
        .name = "myplugin", /* For use in the dump file */
    
        /* Predicate (boolean) function that gets executed before your pass.  If the
         * return value is 'true' your pass gets executed, otherwise, the pass is
         * skipped.
         */
        .gate = [](){return true;},  /* always returns true, see full code */
        .execute = myplugin_exec, /* Your pass handler/callback */
        }
    };


int plugin_init (struct plugin_name_args *plugin_info,
             struct plugin_gcc_version *version)
{
  const char *plugin_name = plugin_info->base_name;
  register_callback(plugin_name,
        PLUGIN_FINISH_DECL, &finish_type_callback, 0);
  register_callback (plugin_name, PLUGIN_ATTRIBUTES, register_attributes, NULL);
struct register_pass_info pass;
   pass.pass = &myplugin_pass.pass;
    
        /*
	 * Get called after GCC has produced the SSA representation of the program.
         * After the first SSA pass.
         */
        pass.reference_pass_name = "ssa";
        pass.ref_pass_instance_number = 1;
        pass.pos_op = PASS_POS_INSERT_AFTER;
    
        /* Tell GCC we want to be called after the first SSA pass */
        register_callback("myplugin", PLUGIN_PASS_MANAGER_SETUP, NULL, &pass);
  return 0;
}
