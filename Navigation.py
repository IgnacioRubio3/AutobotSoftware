# Get input from the user
user_input = input("Enter a number to double: ")

# Convert the input to a number and multiply by 2
try:
    number = float(user_input)
    result = number * 2
    print(f"The double of {number} is {result}")
except ValueError:
    print("That's not a valid number!")