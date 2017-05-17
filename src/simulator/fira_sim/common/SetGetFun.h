#define DECLARE_GET_FUNCTION(func_name, var_type, var_name)\
	var_type Get##func_name##()\
    {\
        return this->var_name;\
    }

#define DECLARE_SET_GET_FUNCTION_STR(func_name, var_type, var_name)\
	void Set##func_name##(var_type  var_name)							\
    {																\
		memcpy((void *)this->var_name, (const void *)var_name, sizeof(this->var_name));\
    }\
	DECLARE_GET_FUNCTION(func_name, var_type, var_name)
	

#define DECLARE_SET_GET_FUNCTION_INT(func_name, var_type, var_name)\
	void Set##func_name##(var_type  var_name)							\
    {																\
		this->var_name = var_name;									\
    }\
	DECLARE_GET_FUNCTION(func_name, var_type, var_name)

