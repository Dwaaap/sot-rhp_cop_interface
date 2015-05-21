#include <boost/function.hpp>

#include <sot/core/unary-op.hh>
#include <sot/core/binary-op.hh>
#include <jrl/mal/boost.hh>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-twist.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>
#include <sot/core/vector-quaternion.hh>

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

#include <dynamic-graph/linear-algebra.h>
#include <sot/core/factory.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/debug.hh>

#include <deque>

namespace dg = ::dynamicgraph;

#define REGISTER_BINARY_OP( OpType,name )				\
  template<>								\
  const std::string BinaryOp< OpType >::CLASS_NAME = std::string(#name); \
  Entity *regFunction##_##name( const std::string& objname )		\
  {									\
    return new BinaryOp< OpType >( objname );				\
  }									\
  EntityRegisterer regObj##_##name( std::string(#name),&regFunction##_##name)

/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/


namespace dynamicgraph {
  namespace sot {
   template< typename TypeRef >
    struct TypeNameHelper
    {
      static const std::string typeName;
    };
    template< typename TypeRef >
    const std::string TypeNameHelper<TypeRef>::typeName = "unspecified";

#define ADD_KNOWN_TYPE( typeid ) \
    template<>const std::string TypeNameHelper<typeid>::typeName = #typeid

    ADD_KNOWN_TYPE(double);
    ADD_KNOWN_TYPE(dg::Vector);
    ADD_KNOWN_TYPE(dg::Matrix);
    ADD_KNOWN_TYPE(MatrixRotation);
    ADD_KNOWN_TYPE(MatrixTwist);
    ADD_KNOWN_TYPE(MatrixHomogeneous);
    ADD_KNOWN_TYPE(VectorQuaternion);
    ADD_KNOWN_TYPE(VectorRollPitchYaw);

 
    template< typename TypeIn1,typename TypeIn2, typename TypeOut >
    struct BinaryOpHeader
    {
      typedef TypeIn1 Tin1;
      typedef TypeIn2 Tin2;
      typedef TypeOut Tout;
      static const std::string & nameTypeIn1(void) { return TypeNameHelper<Tin1>::typeName; }
      static const std::string & nameTypeIn2(void) { return TypeNameHelper<Tin2>::typeName; }
      static const std::string & nameTypeOut(void) { return TypeNameHelper<Tout>::typeName; }
      void addSpecificCommands(Entity&, Entity::CommandMap_t& ) {}
      virtual std::string getDocString () const
      {
	return std::string
	  ("Undocumented binary operator\n"
	   "  - input  ") + nameTypeIn1 () +
	  std::string ("\n"
	   "  -        ") + nameTypeIn2 () +
	  std::string ("\n"
		       "  - output ") + nameTypeOut () +
	  std::string ("\n");
      }
    };

    /*--------Add for RH CoP -----------*/

    struct EntityToMatrix
      : public BinaryOpHeader<ml::Vector, ml::Vector, ml::Vector>
    {
      void operator()( const ml::Vector& v1,const ml::Vector& v2,ml::Vector& res ) const  
      {
        res.resize(6);
	      for( int i=0;i<3;++i ){
	        res(i) = v1(i);
	      }

        for( int i=0;i<3;++i ){
          res(i+3) = v2(i);
        }
      }
    };

    REGISTER_BINARY_OP(EntityToMatrix, EntityToMatrix);

 
/* 
     struct EntityMatrixCompute
      : public BinaryOpHeader<ml::Vector, MatrixHomogeneous, ml::Vector>
    {
      void operator()( const ml::Vector& m1,const MatrixHomogeneous& m2,ml::Vector& res ) const 
      { 
	      ml::Vector v1(4); 
	      v1(3) = 1;
	      ml::Vector v2(3);

	      for( int i=0;i<3;++i ){
	        v1(i) = m1(i);
	      }
	      for( int i=0;i<3;++i ){
	        v2(i) = m1(i+3);
	      }

	      
	      ml::Vector tmp1(4); 
	      m2.multiply(v1,tmp1);	      
	      ml::Vector tmp2(3); 
	      MatrixRotation r; 
	      m2.extract(r);

	      r.multiply(v2, tmp2);
	      res.resize(6);

	      for( int i=0;i<3;++i ){
	        res(i) = v1(i);
	      }
	      for( int i=0;i<3;++i ){
	        res(i+3) = v2(i);
        }
      }
 
   };
    
    REGISTER_BINARY_OP(EntityMatrixCompute,EntityMatrixCompute);
*/
    /*--------Add for RH CoP -----------*/
} /* namespace sot */} /* namespace dynamicgraph */



