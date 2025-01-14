#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os

# import dotenv
from langchain_openai import ChatOpenAI

def get_llm(streaming: bool = False):

    # load environment variables from .env
    # dotenv.load_dotenv(dotenv.find_dotenv())

    # Retrieve the OpenAI API key from environment variables (Nursultan key)
    #OPENAI_API_KEY = "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" # Added
    
    #Dayna's Key
    OPENAI_API_KEY = "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"
    # Initialize the OpenAI Chat model using the API key
    llm = ChatOpenAI(
        model_name="gpt-4o",  # Specify the model (you can change this to gpt-4 or others)
        openai_api_key=OPENAI_API_KEY,  # Provide the OpenAI API key for authentication
        streaming=streaming,  # Optionally enable streaming for larger responses
    )
    
    return llm


def get_env_variable(var_name: str) -> str:
    """
    Retrieves the value of the specified environment variable.

    Args:
        var_name (str): The name of the environment variable to retrieve.

    Returns:
        str: The value of the environment variable.

    Raises:
        ValueError: If the environment variable is not set.

    This function provides a consistent and safe way to retrieve environment variables.
    By using this function, we ensure that all required environment variables are present
    before proceeding with any operations. If a variable is not set, the function will
    raise a ValueError, making it easier to debug configuration issues.
    """
    value = os.getenv(var_name) # Retrieve the environment variable's value
    if value is None: # If the environment variable is not set, raise an error
        raise ValueError(f"Environment variable {var_name} is not set.")
    return value