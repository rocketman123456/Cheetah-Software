#pragma once

class BridgeInterface
{
public:
    virtual void initialize() = 0;
    virtual void finalize() = 0;

    virtual void update() = 0;
};
