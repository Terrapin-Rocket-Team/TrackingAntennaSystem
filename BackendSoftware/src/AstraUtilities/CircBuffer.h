template <typename T>
class CircBuffer
{
    public:
        // constructors & destructors
        CircBuffer(int size);
        ~CircBuffer();
        CircBuffer(const CircBuffer &other); // we use this as a copy constructor

        // copy assignment operator
        CircBuffer &operator=(const CircBuffer &other);

        // methods relevant to data structure
        void push(T item);
        T pop();
        T peek();
        int getCount();
        int getSize();
        bool isFull();
        bool isEmpty();
        void clear();

        // one liners
        T &operator[](int index); 
        T operator[](int index) const;

    private:
        // fields
        T *buffer;
        int size = 0;
        int head = 0;
        int tail = 0;
        int count = 0;

};