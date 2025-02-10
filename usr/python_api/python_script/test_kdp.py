# import torch
import numpy

def test_func(a):
	# a = torch.rand(3,3)
	b = numpy.ones((a,a))
	b = b*3
	b[0,0] = 3.896
	print(b)
	return b