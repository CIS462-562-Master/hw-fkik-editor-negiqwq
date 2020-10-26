#include "aActor.h"
#include <algorithm>

#pragma warning(disable : 4018)



/****************************************************************
*
*    	    Actor functions
*
****************************************************************/

AActor::AActor() 
{
	m_pInternalSkeleton = new ASkeleton();
	m_pSkeleton = m_pInternalSkeleton;

	m_BVHController = new BVHController();
	m_BVHController->setActor(this);

	m_IKController = new IKController();
	m_IKController->setActor(this);

	// code to update additional Actor data goes here
	resetGuide();

}

AActor::AActor(const AActor* actor)
{
	*this = *actor;
}

AActor& AActor::operator = (const AActor& actor)
{
	// Performs a deep copy
	if (&actor == this)
	{
		return *this;
	}
	m_pSkeleton = actor.m_pSkeleton;

	// code to update additional Actor data goes here


	return *this;
}

AActor::~AActor()
{
	 delete m_IKController;
	 delete m_BVHController;
	 delete m_pInternalSkeleton;

}

void AActor::clear()
{
	// looks like it is clearing more times than the number of actors.  as a result, m_pSkeleton is not defined for last case.
	m_pSkeleton->clear();  

	// code to update additional Actor data goes here
}

void AActor::update()
{
	if (!m_pSkeleton->getRootNode() )
		 return; // Nothing loaded
	else m_pSkeleton->update();

	// code to update additional Actor data goes here

}

ASkeleton* AActor::getSkeleton()
{
	return m_pSkeleton;
}

void AActor::setSkeleton(ASkeleton* pExternalSkeleton)
{
	m_pSkeleton = pExternalSkeleton;
}

void AActor::resetSkeleton()
{
	m_pSkeleton = m_pInternalSkeleton;
}

BVHController* AActor::getBVHController()
{
	return m_BVHController;
}

IKController* AActor::getIKController()
{
	return m_IKController;
}

void AActor::updateGuideJoint(vec3 guideTargetPos)
{
	if (!m_pSkeleton->getRootNode()) { return; }

	// TODO: 
	// 1.	Set the global position of the guide joint to the global position of the root joint-
	// 2.	Set the y component of the guide position to 0
	// 3.	Set the global rotation of the guide joint towards the guideTarget
	vec3 root_trans = m_pSkeleton->getRootNode()->getGlobalTranslation();
	vec3 guide_trans = vec3(root_trans[0], 0, root_trans[2]);
	m_Guide.setLocalTranslation(guide_trans);

	
	vec3 rot_x_axis{ guideTargetPos[0] - guide_trans[0], 0, guideTargetPos[2] - guide_trans[2] };
	vec3 rot_y_axis{ 0, 1, 0 };
	vec3 rot_z_axis = rot_x_axis.Cross(rot_y_axis);
	mat3 rot_matrix{ rot_x_axis, rot_y_axis, rot_z_axis };
	m_Guide.setGlobalRotation(rot_matrix);
}

void AActor::solveFootIK(float leftHeight, float rightHeight, bool rotateLeft, bool rotateRight, vec3 leftNormal, vec3 rightNormal)
{
	if (!m_pSkeleton->getRootNode()) { return; }
	AJoint* leftFoot = m_pSkeleton->getJointByID(m_IKController->mLfootID);
	AJoint* rightFoot = m_pSkeleton->getJointByID(m_IKController->mRfootID);

	// TODO: 
	// The normal and the height given are in the world space

	// 1.	Update the local translation of the root based on the left height and the right height
	m_pSkeleton->getRootNode()->setLocalTranslation(vec3(0, std::min(leftHeight, rightHeight), 0));
	m_pSkeleton->update();

	// 2.	Update the character with Limb-based IK 
	vec3 left_pos = leftFoot->getGlobalTranslation();
	vec3 right_pos = rightFoot->getGlobalTranslation();
	ATarget left_tar = ATarget();
	left_tar.setGlobalTranslation(vec3(left_pos[0], leftHeight, left_pos[2]));
	ATarget right_tar = ATarget();
	right_tar.setGlobalTranslation(vec3(right_pos[0], rightHeight, right_pos[2]));
	m_IKController->IKSolver_Limb(leftFoot->getID(), left_tar);
	m_IKController->IKSolver_Limb(rightFoot->getID(), right_tar);
	
	// Rotate Foot
	if (rotateLeft)
	{
		// Update the local orientation of the left foot based on the left normal
		mat3 current_left_ori = leftFoot->getGlobalRotation();
		mat3 new_left_ori{ leftNormal.Cross(current_left_ori[2]), leftNormal, current_left_ori[2] };
		leftFoot->setGlobalRotation(new_left_ori);
	}
	if (rotateRight)
	{
		// Update the local orientation of the right foot based on the right normal
		mat3 current_right_ori = rightFoot->getGlobalRotation();
		mat3 new_right_ori{ rightNormal.Cross(current_right_ori[2]), rightNormal, current_right_ori[2] };
		rightFoot->setGlobalRotation(new_right_ori);
	}
	m_pSkeleton->update();
}
