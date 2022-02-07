from bs4 import BeautifulSoup, NavigableString

def add_message(soup, message):
    span = soup.find('span',id='typed')
    span.insert(0, NavigableString(message))
    return soup

def generate_table(list):
    table = "<table>\n"

    # Create the table's column headers
    header = list[0].split(",")
    table += "  <tr>\n"
    for column in header:
        table += "    <th>{0}</th>\n".format(column.strip())
    table += "  </tr>\n"

    # Create the table's row data
    for line in list[1:]:
        row = line.split(",")
        table += "  <tr>\n"
        for column in row:
            table += "    <td>{0}</td>\n".format(column.strip())
        table += "  </tr>\n"

    table += "</table>"
    return table

def add_table(soup, list):
    if list:
        flex_container = soup.find("div", id='container')
        flex_item = "<div class=\"row\"><div class=\"flex-item\"><span id=\"table\"></span></div></div>"
        flex_container.append(BeautifulSoup(flex_item, 'html.parser'))
        span_table = soup.find('span',id='table')
        table = generate_table(list)
        span_table.append(BeautifulSoup(table, features="html.parser"))

    return soup

class HtmlGenerator:

    def __init__(self, path_template, path_output):
        self.path_output_folder = path_output
        self.path_template = path_template
        

    def get_template(self):
        with open(self.path_template, 'r') as template:
            return BeautifulSoup(template, 'html.parser')

    def generate_html(self, message, product_list):
        tmp_soup = self.get_template()
        tmp_soup = add_message(tmp_soup, message)
        tmp_soup = add_table(tmp_soup, product_list)
        with open(self.path_output_folder, 'w') as generated_html:
            generated_html.write(tmp_soup.prettify())


