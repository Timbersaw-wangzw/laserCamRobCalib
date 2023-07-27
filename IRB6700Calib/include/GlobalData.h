#pragma once

enum RegularizationName { non, L1, L2, ElasticNet, Log, Swish };

struct _4MCPCParas {
	double alpha;
	double beta;
	double x;
	double y;
};


struct _6MCPCParas {
	double alpha;
	double beta;
	double gamma;
	double x;
	double y;
	double z;
};

_4MCPCParas operator+(_4MCPCParas p1, _4MCPCParas p2);

_6MCPCParas operator+(_6MCPCParas p1, _6MCPCParas p2);

inline _4MCPCParas operator+(_4MCPCParas p1, _4MCPCParas p2)
{
	_4MCPCParas p;
	p.alpha = p1.alpha + p2.alpha;
	p.beta = p1.beta + p2.beta;
	p.x = p1.x + p2.x;
	p.y = p1.y + p2.y;
	return p;
}

inline _6MCPCParas operator+(_6MCPCParas p1, _6MCPCParas p2)
{
	_6MCPCParas p;
	p.alpha = p1.alpha + p2.alpha;
	p.beta = p1.beta + p2.beta;
	p.gamma = p1.gamma + p2.gamma;
	p.x = p1.x + p2.x;
	p.y = p1.y + p2.y;
	p.z=p1.z+p2.z;
	return p;
}