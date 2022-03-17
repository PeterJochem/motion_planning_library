#include <iostream>
#include <fcl/fcl.h>

using namespace fcl;
void loadOBJFile(const char* filename, std::vector<Vector3f>& points, std::vector<Triangle>& triangles)
{

  FILE* file = fopen(filename, "rb");
  if(!file)
  {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  bool has_normal = false;
  bool has_texture = false;
  char line_buffer[2000];
  while(fgets(line_buffer, 2000, file))
  {
    char* first_token = strtok(line_buffer, "\r\n\t ");
    if(!first_token || first_token[0] == '#' || first_token[0] == 0)
      continue;

    switch(first_token[0])
    {
    case 'v':
      {
        if(first_token[1] == 'n')
        {
          strtok(nullptr, "\t ");
          strtok(nullptr, "\t ");
          strtok(nullptr, "\t ");
          has_normal = true;
        }
        else if(first_token[1] == 't')
        {
          strtok(nullptr, "\t ");
          strtok(nullptr, "\t ");
          has_texture = true;
        }
        else
        {
          float x = (float)atof(strtok(nullptr, "\t "));
          float  y = (float)atof(strtok(nullptr, "\t "));
          float  z = (float)atof(strtok(nullptr, "\t "));
          points.emplace_back(x, y, z);
        }
      }
      break;
    case 'f':
      {
        Triangle tri;
        char* data[30];
        int n = 0;
        while((data[n] = strtok(nullptr, "\t \r\n")) != nullptr)
        {
          if(strlen(data[n]))
            n++;
        }

        for(int t = 0; t < (n - 2); ++t)
        {
          if((!has_texture) && (!has_normal))
          {
            tri[0] = atoi(data[0]) - 1;
            tri[1] = atoi(data[1]) - 1;
            tri[2] = atoi(data[2]) - 1;
          }
          else
          {
            const char *v1;
            for(int i = 0; i < 3; i++)
            {
              // vertex ID
              if(i == 0)
                v1 = data[0];
              else
                v1 = data[t + i];

              tri[i] = atoi(v1) - 1;
            }
          }
          triangles.push_back(tri);
        }
      }
    }
  }
}


int main() {
	
	using namespace fcl;

	std::cout << "Hello World" << std::endl;
	
	
	// set mesh triangles and vertice indices
	std::vector<Vector3f> vertices;
	std::vector<Triangle> triangles;
	// code to set the vertices and triangles
	// BVHModel is a template class for mesh geometry, for default OBBRSS template
	// is used
	
	typedef BVHModel<OBBRSSf> Model;
	std::shared_ptr<Model> geom = std::make_shared<Model>();
	
	loadOBJFile("/home/pj/Desktop/MotionPlannning/fcl/test/fcl_resources/rob.obj", vertices, triangles);
  	//test::loadOBJFile(TEST_RESOURCES_DIR"/rob.obj", p2, t2);	
		

	// add the mesh data into the BVHModel structure
	geom->beginModel();
	geom->addSubModel(vertices, triangles);
	geom->endModel();
	
	// R and T are the rotation matrix and translation vector
	Matrix3f R;
	Vector3f T = {0,0,0};
	
	Transform3f pose = Transform3f::Identity();
	//pose.linear() = R;
	//pose.translation() = T;
	
	//Combine them together
	CollisionObjectf* obj1 = new CollisionObjectf(geom, pose);
	CollisionObjectf* obj2 = new CollisionObjectf(geom, pose);
	
	
	CollisionRequestf request;
	
	CollisionResultf result;
	collide(obj1, obj2, request, result);
	
	if (result.isCollision()) {
		std::cout << "The objects are in collision" << std::endl;
	}
	else {
		std::cout << "The objects are not in collision" << std::endl;
	}
	

	return 0;
}

