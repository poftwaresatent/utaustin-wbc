/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2009 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

//==============================================================================
/*!
  \author			Luis Sentis
  \file				BranchingRepresentation.h
*/
//==============================================================================

#ifndef WBC_Branching_Representation_H
#define WBC_Branching_Representation_H

#include <jspace/tao_util.hpp>
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <tao/matrix/TaoDeMath.h>	
#include <map>
#include <string>
#include <vector>

class taoDNode;
class taoNode;	
class taoNodeRoot;

namespace wbc {
  
  using jspace::idToNodeMap_t;
  using namespace std;
  
  
  /*!
    \brief An entity that provides the support to build a branching representation
    of TaoNodes.
  */

  class BranchingRepresentation {
    friend class TiXmlBRParser;
    friend class OsimBRParser;
    friend class BRBuilder;
    BranchingRepresentation();

  public:
    /**
       Create a BranchingRepresentation from an existing TAO
       tree. Given that, in some sense, BranchingRepresentation just
       adds a little frosting on top of TAO, this is actually a fairly
       easy way to create one, as long as you do not need some of the
       fancier features like defaultJointPositions() or
       surfaceDepth().
    */
    static BranchingRepresentation *
    create(/** root of a valid and fully populated TAO tree */
	   taoNodeRoot * tao_root,
	   /** optional gravity vector (if NULL, we fall to Earth along negative Z) */
	   SAIVector const * opt_gravity,
	   /** optional un-actuation matrix (if NULL, we construct an identity matrix for it) */
	   SAIMatrix const * opt_unactuation_matrix,
	   /** optional name for the root link (if empty, we set it to "root") */
	   std::string const & opt_root_name,
	   /** optional array of link names (if non-NULL, we pass it to setLinkNames()) */
	   std::vector<std::string> const * opt_link_names,
	   /** optional array of link names (if non-NULL, we pass it to setJointNames()) */
	   std::vector<std::string> const * opt_joint_names,
	   /** optional array of lower joint limits */
	   std::vector<double> const * opt_joint_limits_lower,
	   /** optional array of upper joint limits */
	   std::vector<double> const * opt_joint_limits_upper)
      throw(std::runtime_error);
    
    
    ~BranchingRepresentation();
    
    /** Set the name of the TAO root node, simply overriding any
	previously existing one. It also asks canonicalLinkName() for
	a string, and adds that as well. */
    void setRootName(std::string const & root_name) throw(std::runtime_error);
    
    /** Set the names of the non-root links, by using a table that
	relates node IDs to strings. It also asks canonicalLinkName()
	for a string, and adds that as well, simply overriding any
	previously existing link names. */
    void setLinkNames(std::vector<std::string> const & link_names) throw(std::runtime_error);
    
    /** Set the names of the joints, by using a table that relates the
	IDs of the nodes for which the joint is the parent to
	strings. It also asks canonicalJointName() for a string, and
	adds that as well, simply overriding any previously existing
	joint names.
	
	\note TAO supports multiple joints per link, but WBC never
	used that feature. So, in our context, it actually makes sense
	to use a \c std::vector and treat the index as node ID.
    */
    void setJointNames(std::vector<std::string> const & joint_names) throw(std::runtime_error);
    
    /** Loops over all registered link and joint names and injects
	them as well as their canonical counterparts into
	linkNameToNodeMapWithAliases_ and
	jointNameToNodeMapWithAliases_. This is called lazily from
	within linkNameToNodeMap() and jointNameToNodeMap() when you
	call it with \c withAliases=true (which is the default). */
    void createCanonicalAliases() const;
    
    inline taoNodeRoot* rootNode() {return rootNode_; };  
  
    inline idToNodeMap_t & idToNodeMap() { return idToNodeMap_; }
    inline taoDNode* node(int nodeID) { return idToNodeMap_[nodeID];}
    
    std::map<std::string, taoDNode*> const & linkNameToNodeMap(bool withAliases = true) const;
    std::map<std::string, taoDNode*> const & jointNameToNodeMap(bool withAliases = true) const;
    
    taoDNode * findLink( std::string const & name );
    taoDNode * findJoint( std::string const & name );
    
    inline int numJoints() const {return numJoints_;}
    inline int numActuatedJoints() const {return unactuationMatrix_.row();}
  	  
    inline const SAIVector& defaultJointPositions() const { return defaultJointPosVec_; } 
    inline const SAIVector& upperJointLimits() const { return upperJointLimitVec_; } 
    inline const SAIVector& lowerJointLimits() const { return lowerJointLimitVec_; } 

    double surfaceDepth( taoDNode const * node ) const;

    inline double totalMass() const { return totalMass_; }
    inline const SAIVector& gravityAcceleration() const { return grav_; }
  
    inline const SAIMatrix& unactuationMatrix() const { return unactuationMatrix_; }

    /** \return relative position of force sensor. Returns zero vector
	      if no link tag is found */
    const SAIVector& forceSensorWRTJointFrame( taoDNode const * node ) const;
    
    /** Conversion function for easier back-porting to newer code. You
	should try to work with a jspace::tao_tree_info_s in the first
	place... */
    jspace::tao_tree_info_s * createTreeInfo();
    
  private:
    // Attributes 
    taoNodeRoot* rootNode_; 
    idToNodeMap_t idToNodeMap_; 
    std::map<std::string, taoDNode*> linkNameToNodeMap_;
    std::map<std::string, taoDNode*> jointNameToNodeMap_;
    mutable std::map<std::string, taoDNode*> linkNameToNodeMapWithAliases_;
    mutable std::map<std::string, taoDNode*> jointNameToNodeMapWithAliases_;
    std::map<taoDNode const *, SAIVector> linkToSensorMap_;
    std::map<taoDNode const *, double> linkToSurfaceDepthMap_;
    int numJoints_;
    double totalMass_;
    SAIVector grav_;
    SAIMatrix unactuationMatrix_;

    // Constant
    static const SAIVector zero3;
  
    // Supporting attributes
    deVector3 pos_;
    SAIVector defaultJointPosVec_;
    SAIVector upperJointLimitVec_; //hard jointLimits
    SAIVector lowerJointLimitVec_; //hard jointLimits
    int parentNodeID_;

    // Private functions
    
    /** \return tao node from ID. */
    taoDNode* findNodeID( taoDNode*, int);

    static std::string __canonicalJointName( const std::string & );
    static std::string __canonicalLinkName( const std::string & );
  };

}

#endif // WBC_Branching_Representation_H
