#include "Bone_Animation.h"
#include <glm/gtx/euler_angles.hpp>

/*****************************************************************************
        * Written by Srivastchavan Rengarajan starting November 09, 2020.
******************************************************************************/


Bone_Animation::Bone_Animation()
{
}


Bone_Animation::~Bone_Animation()
{
}

void Bone_Animation::init()
{
	root_position = { 2.0f,0.5f,2.0f };

    target = { 3.0f,8.0f,3.0f };

    target_colors = { 0.0f, 1.0f, 0.0f, 1.0f };

	scale_vector =
	{
		{1.0f,1.0f,1.0f},
		{0.5f,4.0f,0.5f},
		{0.5f,3.0f,0.5f},
		{0.5f,2.0f,0.5f}
	};

	rotation_degree_vector = 
	{
		{0.0f,0.0f,0.0f},
		{0.0f,30.0f,0.0f},
		{0.0f,30.0f,0.0f},
		{0.0f,30.0f,0.0f}
	};

	colors = 
	{
		{0.7f,0.0f,0.0f,1.0f},
		{0.7f,0.7f,0.0f,1.0f},
		{0.7f,0.0f,0.7f,1.0f},
		{0.0f,0.7f,0.7f,1.0f}
	};

    bone_mat = { glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f) };

    isMoving = false;
    tree_depth = (int)scale_vector.size();
    joint_num = tree_depth - 1;

    Rotation();
}

void Bone_Animation::update(float delta_time)
{

	pivot.clear();
	X_axis.clear();
	Y_axis.clear();
	Z_axis.clear();

	Rotation();

	if (isMoving)
	{
		JacobianTranspose();
	}
}


void Bone_Animation::Rotation()
{
    int i,j;

    std::vector<float> X_angle = { glm::radians(rotation_degree_vector[0][2]), glm::radians(rotation_degree_vector[1][2]), glm::radians(rotation_degree_vector[2][2]), glm::radians(rotation_degree_vector[3][2]) };

    std::vector<float> Y_angle = { glm::radians(rotation_degree_vector[0][0]), glm::radians(rotation_degree_vector[1][0]), glm::radians(rotation_degree_vector[2][0]), glm::radians(rotation_degree_vector[3][0]) };

    std::vector<float> Z_angle = { glm::radians(rotation_degree_vector[0][1]), glm::radians(rotation_degree_vector[1][1]), glm::radians(rotation_degree_vector[2][1]), glm::radians(rotation_degree_vector[3][1]) };

    rotateX = { glm::eulerAngleX(X_angle[0]), 
                glm::eulerAngleX(X_angle[1]), 
                glm::eulerAngleX(X_angle[2]), 
                glm::eulerAngleX(X_angle[3]) };

    rotateY = { glm::eulerAngleX(Y_angle[0]), 
                glm::eulerAngleY(Y_angle[1]), 
                glm::eulerAngleY(Y_angle[2]), 
                glm::eulerAngleY(Y_angle[3]) };

    rotateZ = { glm::eulerAngleX(Z_angle[0]), 
                glm::eulerAngleZ(Z_angle[1]), 
                glm::eulerAngleZ(Z_angle[2]), 
                glm::eulerAngleZ(Z_angle[3]) };

    for ( i = 0; i < tree_depth; i++)
    {
        j = i - 1;
        rotate_mat[i] = rotateX[i] * rotateZ[i] * rotateY[i];

        if (i == 0)
        {
            translate_mat_origin[i] = glm::translate(glm::mat4(1.0f), { 0, scale_vector[i][1] / 2.0f, 0 });
            translate_mat_link[i] = glm::translate(glm::mat4(1.0f), { root_position[0], (root_position[1] - scale_vector[i][1] / 2.0f),root_position[2] });
            translate_mat_end[i] = glm::translate(glm::mat4(1.0f), { 0, scale_vector[i][1], 0 });
            world_mat[i] = translate_mat_link[i] * rotate_mat[i];
            bone_mat[i] = glm::translate(glm::mat4(1.0f), root_position);
        }
        else
        {
            translate_mat_origin[i] = glm::translate(glm::mat4(1.0f), { 0, scale_vector[i][1] / 2.0f, 0 });
            translate_mat_link[i] = glm::translate(glm::mat4(1.0f), { 0, scale_vector[i - 1][1], 0 });
            translate_mat_end[i] = glm::translate(glm::mat4(1.0f), { 0, scale_vector[i][1], 0 });
            world_mat[i] = world_mat[i - 1] * translate_mat_link[i] * rotate_mat[i];
            bone_mat[i] = world_mat[i] * translate_mat_origin[i];

            pivot.push_back(glm::vec3(world_mat[j] * translate_mat_end[j] * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)));
            X_axis.push_back(normalize(glm::vec3(world_mat[j] * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f))));
            Y_axis.push_back(normalize(glm::vec3(world_mat[j] * rotateX[i] * rotateZ[i] * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f))));
            Z_axis.push_back(normalize(glm::vec3(world_mat[j] * rotateX[i] * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f))));
            
        }

    }
    endEffector = world_mat[3] * translate_mat_end[3] * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    distance = glm::distance(target, endEffector);
}


void Bone_Animation::reset()
{
    isMoving = false;

    rotation_degree_vector =
    {
        {0.0f,0.0f,0.0f},
        {0.0f,30.0f,0.0f},
        {0.0f,30.0f,0.0f},
        {0.0f,30.0f,0.0f}
    };

    bone_mat = { glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f) };
}

void Bone_Animation::JacobianTranspose()
{
    
    std::vector<glm::mat3> Jacobian_mat;
    std::vector<glm::mat3> Jacobian_trans                   ;
    std::vector<glm::vec3> deltaTheta;
    std::vector<glm::vec3> alpha_numerator;
    glm::vec3 alpha_denominator;

    if (distance >= threshold) {

        for (int i = 0; i < joint_num; i++)
        {
            
            Jacobian_mat.push_back(glm::mat3(cross(X_axis[i], (endEffector - pivot[i])), 
                                             cross(Y_axis[i], (endEffector - pivot[i])), 
                                             cross(Z_axis[i], (endEffector - pivot[i]))));

            Jacobian_trans.push_back(glm::transpose(Jacobian_mat[i]));

            alpha_numerator.push_back(Jacobian_trans[i] * (target - endEffector));
        }

        alpha_denominator = Jacobian_mat[0] * alpha_numerator[0] + 
                            Jacobian_mat[1] * alpha_numerator[1] + 
                            Jacobian_mat[2] * alpha_numerator[2];

        alpha = (dot(alpha_numerator[0], alpha_numerator[0]) + 
                 dot(alpha_numerator[1], alpha_numerator[1]) + 
                 dot(alpha_numerator[2], alpha_numerator[2]))
                 / (dot(alpha_denominator, alpha_denominator));

        for (int i = 0; i < joint_num; i++)
        {
            deltaTheta.push_back(alpha * Jacobian_trans[i] * (target - endEffector));
            rotation_degree_vector[i + 1] = rotation_degree_vector[i + 1] + glm::vec3(deltaTheta[i][1], deltaTheta[i][2], deltaTheta[i][0]);
        }
    }
}

