#!/usr/bin/env python3

def class_name(class_obj):
    """
    Print class name in a fancy way.
    Parameters:
        - Class object
    """
    WHITESPACE = 10
    SYMBOL     = 2
    name = class_obj.__class__.__name__
    total_length = SYMBOL + WHITESPACE + len(name) + WHITESPACE + SYMBOL

    # Print on terminal
    print('#'*total_length)
    print('#'*SYMBOL + ' '*(total_length-2*SYMBOL) + '#'*SYMBOL)
    print('#'*SYMBOL + ' '*WHITESPACE + class_obj.__class__.__name__ + ' '*WHITESPACE  + "#"*SYMBOL)
    print('#'*SYMBOL + ' '*(total_length-2*SYMBOL) + '#'*SYMBOL)
    print('#'*total_length)
