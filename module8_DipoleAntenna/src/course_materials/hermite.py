#!/usr/bin/env python3
#/****************************************************************************
# Hermite spline example
# Copyright (c) 2016-2020, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# http://sdu.dk/uas
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
'''
2016-10-04 Kjeld Jensen, first version
2020-03-07 Kjeld Jensen, Python3 compatible 
'''

# implementation of a cubic hermite spline

# load modules
from math import pi
import matplotlib.pyplot as plt
from pylab import *



class cubic_hermite_spline():
	def __init__(self):
		pass

	def v2d_scalar_mul (self, v, f):
		return [v[0]*f, v[1]*f]

	def v2d_add (self, v1, v2):
		return [v1[0]+v2[0], v1[1]+v2[1]]

	def goto_wpt (self, p1, t1, p2, t2, steps):
		# http://cubic.org/docs/hermite.htm
		# http://en.wikipedia.org/wiki/Cubic_Hermite_spline#Interpolation%20on%20a%20single%20interval
		p = []
		for t in range(steps):
			s = t/(steps * 1.0) # scale s to go from 0 to 1

			# # calculate basis function
			h1 = 2*s**3 - 3*s**2 + 1
			h2 = -2*s**3 + 3*s**2
			h3 = s**3 - 2*s**2 + s
			h4 = s**3 - s**2

			# multiply and sum functions together to build the interpolated point along the curve
			v1 = self.v2d_scalar_mul(p1,h1)
			v2 = self.v2d_scalar_mul(p2,h2)
			v3 = self.v2d_scalar_mul(t1,h3)
			v4 = self.v2d_scalar_mul(t2,h4)

			p.append(self.v2d_add(self.v2d_add(self.v2d_add(v1,v2),v3),v4))
		return p

	# Function to calculate tangents dynamically
	def calculate_tangents(self, points, k=0.5):
		tangents = []
		n = len(points)
		for i in range(n):
			prev = points[i - 1] if i > 0 else points[-1]  # Wrap around for the first point
			next_ = points[(i + 1) % n]  # Wrap around for the last point
			tangent = [k * (next_[0] - prev[0]), k * (next_[1] - prev[1])]
			tangents.append(tangent)
		return tangents

# Define the points (example: square vertices)
points = [[0, 0], [1, 0], [1, 1], [0, 1]]  # Replace with dynamic points if needed

# Initialize the spline generator
spline = cubic_hermite_spline()

# Calculate tangents
k = 0.5  # Scaling factor for tangent length
tangents = spline.calculate_tangents(points, k)

# Generate the smooth curve
smooth_curve = []
steps = 20  # Number of points per edge
for i in range(len(points)):
    p1 = points[i]
    p2 = points[(i + 1) % len(points)]  # Wrap around
    t1 = tangents[i]
    t2 = tangents[(i + 1) % len(points)]  # Wrap around
    smooth_curve += spline.goto_wpt(p1, t1, p2, t2, steps)

# Visualization
x, y = zip(*smooth_curve)
plt.plot(x, y, label="Smoothed Curve")
plt.scatter(*zip(*points), color='red', label="Original Points")
plt.legend()
plt.axis('equal')
plt.title("Smoothed Curve Using Cubic Hermite Splines")
plt.show()
