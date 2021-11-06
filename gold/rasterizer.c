#include "rasterizer.h"
#include <stdlib.h>
#include <stdio.h> //TODO REMOVE
#ifdef __cplusplus
#include <vector>
#endif


#define BITS sizeof(int) * 8 
/* Utility Functions */

/*
 *   Function: min
 *  Function Description: Returns the minimum value of two integers and b.
*/
int min(int a, int b)
{
  // START CODE HERE
  // END CODE HERE
  if (a <= b)
  {
    return a;
  }
  else
  {
    return b;
  }
}

/*
 *   Function: max
 *   Function Description: Returns the maximum value of two integers and b.
*/
int max(int a, int b)
{
  // START CODE HERE
  // END CODE HERE
  if (a >= b)
  {
    return a;
  }
  else
  {
    return b;
  }
}

/*
/   Function: floor_ss
/   Function Description: Returns a fixed point value rounded down to the subsample grid.
*/

/*
from rast_types.h
r_shift; // number of fractional bits in fixed point representation
ss_w_lg2; // log2(subsample_width): numerically equal to # of bits required to encode subsample_width
*/

int floor_ss(int val, int r_shift, int ss_w_lg2)
{
  // START CODE HERE
  // remove r_shift-ss_w_lg2 bits from end of val
  // in other words, mask val with 0xb11111...10000
  int n_bits_to_trunc = r_shift - ss_w_lg2;
  val = val >> n_bits_to_trunc;
  val = val << n_bits_to_trunc;
  return val;

  // END CODE HERE
}

/*
 *  Function: rastBBox_bbox_fix
 *  Function Description: Determine a bounding box for the triangle.
 *  Note that this is a fixed point function.
*/
BoundingBox get_bounding_box(Triangle triangle, Screen screen, Config config)
{
  BoundingBox bbox;

  // START CODE HERE
  int r_shift = config.r_shift;
  int ss = config.ss;
  double ss_i = config.ss_i;
  int ss_w = config.ss_w;
  int ss_w_lg2 = config.ss_w_lg2;

  // initialize bounding box to first vertex

  ColorVertex3D v0 = triangle.v[0];
  // ColorVertex3D v1 = triangle.v[0];
  //  ColorVertex3D v2 = triangle.v[0];
  int mask = 0b11111;

  int current_min_x = v0.x;
  int current_min_y = v0.y;
  int current_max_x = v0.x;
  int current_max_y = v0.y;

  // fprintf(stderr, "do we get here 1 \n");
  // iterate over remaining vertices
  for (int i = 1; i < 3; i++)
  {
    current_min_x = min(current_min_x, triangle.v[i].x);
    current_min_y = min(current_min_y, triangle.v[i].y);
    current_max_x = max(current_max_x, triangle.v[i].x);
    current_max_y = max(current_max_y, triangle.v[i].y);
  }

  // round down to subsample grid

  /////////
  // should use floor_ss function on each current_min and max here
  /////////

  int min_x = current_min_x;
  int min_y = current_min_y;
  int max_x = current_max_x;
  int max_y = current_max_y;

  min_x = floor_ss(min_x, r_shift, ss_w_lg2);
  min_y = floor_ss(min_y, r_shift, ss_w_lg2);
  max_x = floor_ss(max_x, r_shift, ss_w_lg2);
  max_y = floor_ss(max_y, r_shift, ss_w_lg2);

  // clip to screen

  //fprintf(stderr, "do we get here 5 \n");
  int msb = 1 << (BITS - 1);
  //fprintf(stderr, "this is msb %d \n", msb);
  if (min_x < 0)
  {
    fprintf(stderr, "We are here");
    min_x = 0;
  }
  if (min_y < 0)
  {
    fprintf(stderr, "We are here y");
    min_y = 0;
  }

  int screen_shift = 1024 << r_shift;
  int width_shift = 1024 << r_shift - ss_w_lg2;
  //int msb = 1 << (BITS - 1);
  if (max_x > screen_shift)
  {
    max_x = screen_shift;
  }
  if (max_y > screen_shift)
  {
    max_y = screen_shift;
  }

 // fprintf(stderr, "This is min_x after screen and floor %d \n", min_x >> r_shift -2);
 // fprintf(stderr, "This is max_x after screen and floor %d \n", max_x >> r_shift -2);
 // fprintf(stderr, "do we get here 6 \n");
  // check if bbox is valid
  Vertex2D lower_left;
  lower_left.x = min_x;
  lower_left.y = min_y;

  Vertex2D upper_right;
  upper_right.x = max_x;
  upper_right.y = max_y;

  bbox.lower_left = lower_left;
  bbox.upper_right = upper_right;

 // fprintf(stderr, "do we get here 7 \n");
  bool valid = true;
  if (max_x < 0  || max_y < 0)
  {
    valid = false;
  }

  if (min_x > screen_shift || min_y > screen_shift)
  {
    valid = false;
  }

 // fprintf(stderr, "This is valid %d \n", valid);
 // fprintf(stderr, "do we get here 8 \n");
  // END CODE HERE
  bbox.valid = valid;
  return bbox;
}

/*
 *  Function: sample_test
 *  Function Description: Checks if sample lies inside triangle
 *
 *
 */
bool sample_test(Triangle triangle, Sample sample)
{
  bool isHit;

  // START CODE HERE

  //shift to new coordinate system with sample at origin
  int v0_x = triangle.v[0].x - sample.x;
  int v0_y = triangle.v[0].y - sample.y;
  int v1_x = triangle.v[1].x - sample.x;
  int v1_y = triangle.v[1].y - sample.y;
  int v2_x = triangle.v[2].x - sample.x;
  int v2_y = triangle.v[2].y - sample.y;

  // get distance of all 3 new edges
  int dist0 = v0_x * v1_y - v1_x * v0_y;
  int dist1 = v1_x * v2_y - v2_x * v1_y;
  int dist2 = v2_x * v0_y - v0_x * v2_y;

  // is the sample (origin) to the right side of these edges?
  bool b0 = dist0 <= 0;
  bool b1 = dist1 < 0;
  bool b2 = dist2 <= 0;

  // if the sample is to the same side of all edges, record a hit
  isHit = (b0 && b1 && b2); // || (!b0 && !b1 && !b2);
  //for some reason pseudocode only checks if all are on the right. this should be fine, too?

  // END CODE HERE

  return isHit;
}

int rasterize_triangle(Triangle triangle, ZBuff *z, Screen screen, Config config)
{
  int hit_count = 0;

  //Calculate BBox
  BoundingBox bbox = get_bounding_box(triangle, screen, config);

  //Iterate over samples and test if in triangle
  Sample sample;
  for (sample.x = bbox.lower_left.x; sample.x <= bbox.upper_right.x; sample.x += config.ss_i)
  {
    for (sample.y = bbox.lower_left.y; sample.y <= bbox.upper_right.y; sample.y += config.ss_i)
    {

      Sample jitter = jitter_sample(sample, config.ss_w_lg2);
      jitter.x = jitter.x << 2;
      jitter.y = jitter.y << 2;

      Sample jittered_sample;
      jittered_sample.x = sample.x + jitter.x;
      jittered_sample.y = sample.y + jitter.y;

      bool hit = sample_test(triangle, jittered_sample);

      if (hit)
      {
        hit_count++;
        if (z != NULL)
        {
          Sample hit_location;
          hit_location.x = sample.x >> config.r_shift;
          hit_location.y = sample.y >> config.r_shift;

          Sample subsample;
          subsample.x = (sample.x - (hit_location.x << config.r_shift)) / config.ss_i;
          subsample.y = (sample.y - (hit_location.y << config.r_shift)) / config.ss_i;

          Fragment f;
          f.z = triangle.v[0].z;
          f.R = triangle.v[0].R;
          f.G = triangle.v[0].G;
          f.B = triangle.v[0].B;

          process_fragment(z, hit_location, subsample, f);
        }
      }
    }
  }

  return hit_count;
}

void hash_40to8(uchar *arr40, ushort *val, int shift)
{
  uchar arr32[4];
  uchar arr16[2];
  uchar arr8;

  ushort mask = 0x00ff;
  mask = mask >> shift;

  arr32[0] = arr40[0] ^ arr40[1];
  arr32[1] = arr40[1] ^ arr40[2];
  arr32[2] = arr40[2] ^ arr40[3];
  arr32[3] = arr40[3] ^ arr40[4];

  arr16[0] = arr32[0] ^ arr32[2];
  arr16[1] = arr32[1] ^ arr32[3];

  arr8 = arr16[0] ^ arr16[1];

  mask = arr8 & mask;
  val[0] = mask;
}

Sample jitter_sample(const Sample sample, const int ss_w_lg2)
{
  long x = sample.x >> 4;
  long y = sample.y >> 4;
  uchar arr40_1[5];
  uchar arr40_2[5];

  long *arr40_1_ptr = (long *)arr40_1;
  long *arr40_2_ptr = (long *)arr40_2;

  ushort val_x[1];
  ushort val_y[1];

  *arr40_1_ptr = (y << 20) | x;
  *arr40_2_ptr = (x << 20) | y;

  hash_40to8(arr40_1, val_x, ss_w_lg2);
  hash_40to8(arr40_2, val_y, ss_w_lg2);

  Sample jitter;
  jitter.x = val_x[0];
  jitter.y = val_y[0];

  return jitter;
}
